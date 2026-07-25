//----------------------------------------------------------------------------
// I8085 Trace - GDB Remote Serial Protocol stub
//
// gdb_stub.cpp - RSP server for interactive debugging via GDB or LLDB
//
// Implements a subset of the GDB Remote Serial Protocol over TCP.
// Supports: register read/write, memory read/write, breakpoints,
// single-step, continue, target description XML.
//
// i8085 registers in DWARF order:
//   B(0,8), C(1,8), D(2,8), E(3,8), H(4,8), L(5,8), M(6,8), A(7,8)
//   SP(8,16), FLAGS(9,8), PSW(10,16), PC(11,16)
//   BC(12,16), DE(13,16), HL(14,16)
//----------------------------------------------------------------------------

#include "gdb_stub.hpp"
#include "i8085_cpu.h"
#include "i8085_io_runtime.h"

#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <string>
#include <set>

//----------------------------------------------------------------------------
// Cross-platform socket layer
//
// On Windows the BSD-style POSIX socket API (read/write/close/poll on a file
// descriptor) is replaced by Winsock2 (recv/send/closesocket/WSAPoll on a
// SOCKET handle). The thin wrappers below let the rest of this file stay
// platform-agnostic.
//----------------------------------------------------------------------------

#ifdef _WIN32
#include <winsock2.h>
#include <ws2tcpip.h>

typedef SOCKET socket_t;

// RAII guard so WSAStartup/WSACleanup bracket every gdb_main return path.
struct WinsockInit {
    WinsockInit() {
        WSADATA wsaData;
        WSAStartup(MAKEWORD(2, 2), &wsaData);
    }
    ~WinsockInit() { WSACleanup(); }
};

static int sock_recv(socket_t fd, void *buf, size_t n) {
    return recv(fd, (char *)buf, (int)n, 0);
}
static int sock_send(socket_t fd, const void *buf, size_t n) {
    return send(fd, (const char *)buf, (int)n, 0);
}
static void sock_close(socket_t fd) { closesocket(fd); }
static int sock_poll_readable(socket_t fd) {
    WSAPOLLFD pfd;
    pfd.fd = fd;
    pfd.events = POLLIN;
    pfd.revents = 0;
    return WSAPoll(&pfd, 1, 0);
}
#else
#include <signal.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include <unistd.h>
#include <poll.h>

typedef int socket_t;
#ifndef INVALID_SOCKET
#define INVALID_SOCKET (-1)
#endif

static int sock_recv(socket_t fd, void *buf, size_t n) {
    return (int)read(fd, buf, n);
}
static int sock_send(socket_t fd, const void *buf, size_t n) {
    return (int)write(fd, buf, n);
}
static void sock_close(socket_t fd) { close(fd); }
static int sock_poll_readable(socket_t fd) {
    struct pollfd pfd;
    pfd.fd = fd;
    pfd.events = POLLIN;
    pfd.revents = 0;
    return poll(&pfd, 1, 0);
}
#endif

//----------------------------------------------------------------------------
// Target description XML (served via qXfer:features:read)
//
// 15 registers in DWARF order, little-endian encoding.
// 8-bit regs: B, C, D, E, H, L, M, A, FLAGS
// 16-bit regs: SP, PSW, PC, BC, DE, HL
//----------------------------------------------------------------------------

// i8085 target description. Only the architecture is named: the custom
// gdb-i8085 supplies the (built-in) i8085 register model, whose g-packet
// layout matches read_reg/write_reg below, so no <feature> is needed here.
// (A stock GDB that does not know the i8085 architecture will ignore this and
//  is not supported; use --gdb-arch=z80 for a stock gdb-multiarch instead.)
static const char *targetXML = "<?xml version=\"1.0\"?>\n"
                               "<!DOCTYPE target SYSTEM \"gdb-target.dtd\">\n"
                               "<target version=\"1.0\">\n"
                               "  <architecture>i8085</architecture>\n"
                               "</target>\n";

//----------------------------------------------------------------------------
// Hex helpers
//----------------------------------------------------------------------------

static const char hexchars[] = "0123456789abcdef";

static int hex_val(char c) {
    if (c >= '0' && c <= '9')
        return c - '0';
    if (c >= 'a' && c <= 'f')
        return c - 'a' + 10;
    if (c >= 'A' && c <= 'F')
        return c - 'A' + 10;
    return -1;
}

static void hex_byte(char *buf, UINT8 val) {
    buf[0] = hexchars[(val >> 4) & 0xF];
    buf[1] = hexchars[val & 0xF];
}

static unsigned long parse_hex(const char *s, const char **endp) {
    unsigned long val = 0;
    while (*s) {
        int d = hex_val(*s);
        if (d < 0)
            break;
        val = (val << 4) | d;
        s++;
    }
    if (endp)
        *endp = s;
    return val;
}

//----------------------------------------------------------------------------
// RSP packet I/O
//----------------------------------------------------------------------------

static bool send_packet(socket_t fd, const std::string &data) {
    UINT8 cksum = 0;
    for (char c : data)
        cksum += (UINT8)c;

    char hdr = '$';
    char trailer[3];
    trailer[0] = '#';
    hex_byte(trailer + 1, cksum);

    if (sock_send(fd, &hdr, 1) != 1)
        return false;
    if (!data.empty()) {
        int n = sock_send(fd, data.c_str(), data.size());
        if (n != (int)data.size())
            return false;
    }
    if (sock_send(fd, trailer, 3) != 3)
        return false;

    // Wait for ACK
    char ack;
    if (sock_recv(fd, &ack, 1) != 1)
        return false;
    return (ack == '+');
}

static std::string recv_packet(socket_t fd) {
    char c;
    for (;;) {
        if (sock_recv(fd, &c, 1) != 1)
            return "";
        if (c == '$')
            break;
        if (c == 0x03) {
            // Ctrl-C async break
            return "\x03";
        }
    }

    std::string data;
    for (;;) {
        if (sock_recv(fd, &c, 1) != 1)
            return "";
        if (c == '#')
            break;
        data += c;
    }

    // Consume 2-char checksum (we trust it)
    char ckbuf[2];
    if (sock_recv(fd, ckbuf, 2) != 2)
        return "";

    // Send ACK
    char ack = '+';
    (void)sock_send(fd, &ack, 1);

    return data;
}

//----------------------------------------------------------------------------
// Fire periodic timers if due
//----------------------------------------------------------------------------

static void fire_timers(State8085 *state, ExecutionStats8085 *stats, std::vector<PeriodicTimer> &timers) {
    for (auto &t : timers) {
        while (stats->total_tstates >= t.nextTriggerCycle) {
            triggerInterrupt(state, t.code, 1);
            t.nextTriggerCycle += t.periodCycles;
        }
    }
}

//----------------------------------------------------------------------------
// Pack flags byte from State8085
//----------------------------------------------------------------------------

static UINT8 pack_flags(const Flags &cc) {
    UINT8 f = 0;
    if (cc.s)
        f |= 0x80;
    if (cc.z)
        f |= 0x40;
    if (cc.ac)
        f |= 0x10;
    if (cc.p)
        f |= 0x04;
    f |= 0x02; // bit 1 always 1
    if (cc.cy)
        f |= 0x01;
    return f;
}

//----------------------------------------------------------------------------
// Unpack flags byte into State8085
//----------------------------------------------------------------------------

static void unpack_flags(State8085 *state, UINT8 f) {
    state->cc.s = (f >> 7) & 1;
    state->cc.z = (f >> 6) & 1;
    state->cc.ac = (f >> 4) & 1;
    state->cc.p = (f >> 2) & 1;
    state->cc.cy = f & 1;
}

//----------------------------------------------------------------------------
// Register access
//
// 15 registers in DWARF order:
//   0:B  1:C  2:D  3:E  4:H  5:L  6:M  7:A
//   8:SP(16)  9:FLAGS(8)  10:PSW(16)  11:PC(16)
//   12:BC(16)  13:DE(16)  14:HL(16)
//
// Register serialization sizes (hex chars):
//   8-bit regs: 2 hex chars      16-bit regs: 4 hex chars (little-endian)
//----------------------------------------------------------------------------

#define NUM_REGS 15

// Returns the size in bytes of register regnum
static int reg_size(int regnum) {
    switch (regnum) {
    case 0:
    case 1:
    case 2:
    case 3:
    case 4:
    case 5:
    case 6:
    case 7:
    case 9:
        return 1; // 8-bit
    case 8:
    case 10:
    case 11:
    case 12:
    case 13:
    case 14:
        return 2; // 16-bit
    default:
        return 0;
    }
}

// Read a register value. For 16-bit regs returns the 16-bit value.
// For 8-bit regs returns the 8-bit value (upper bits zero).
static UINT16 read_reg(State8085 *state, int regnum) {
    UINT8 flags = pack_flags(state->cc);
    switch (regnum) {
    case 0:
        return state->b;
    case 1:
        return state->c;
    case 2:
        return state->d;
    case 3:
        return state->e;
    case 4:
        return state->h;
    case 5:
        return state->l;
    case 6:
        return state->memory[(state->h << 8) | state->l]; // M = (HL)
    case 7:
        return state->a;
    case 8:
        return state->sp;
    case 9:
        return flags;
    case 10:
        return (state->a << 8) | flags; // PSW = A:FLAGS
    case 11:
        return state->pc;
    case 12:
        return (state->b << 8) | state->c; // BC
    case 13:
        return (state->d << 8) | state->e; // DE
    case 14:
        return (state->h << 8) | state->l; // HL
    default:
        return 0;
    }
}

// Write a register value
static void write_reg(State8085 *state, int regnum, UINT16 val) {
    switch (regnum) {
    case 0:
        state->b = val & 0xFF;
        break;
    case 1:
        state->c = val & 0xFF;
        break;
    case 2:
        state->d = val & 0xFF;
        break;
    case 3:
        state->e = val & 0xFF;
        break;
    case 4:
        state->h = val & 0xFF;
        break;
    case 5:
        state->l = val & 0xFF;
        break;
    case 6: // M = (HL)
        state->memory[(state->h << 8) | state->l] = val & 0xFF;
        break;
    case 7:
        state->a = val & 0xFF;
        break;
    case 8:
        state->sp = val;
        break;
    case 9:
        unpack_flags(state, val & 0xFF);
        break;
    case 10: // PSW = A:FLAGS
        state->a = (val >> 8) & 0xFF;
        unpack_flags(state, val & 0xFF);
        break;
    case 11:
        state->pc = val;
        break;
    case 12: // BC
        state->b = (val >> 8) & 0xFF;
        state->c = val & 0xFF;
        break;
    case 13: // DE
        state->d = (val >> 8) & 0xFF;
        state->e = val & 0xFF;
        break;
    case 14: // HL
        state->h = (val >> 8) & 0xFF;
        state->l = val & 0xFF;
        break;
    }
}

//----------------------------------------------------------------------------
// z80 masquerade
//
// GDB has no i8085 architecture, but it does support z80. When enabled (the
// simulator's --gdb-arch=z80 option), the stub presents the CPU as a z80 so a
// stock gdb-multiarch can debug 8085 firmware (relabelled as elf32-z80). GDB's
// z80 register block is 13 x 16-bit regs, in this order:
//   0 af  1 bc  2 de  3 hl  4 sp  5 pc  6 ix  7 iy  8 af' 9 bc' 10 de' 11 hl' 12 ir
// The 8085 registers map onto the main set; the z80-only regs read as zero.
//----------------------------------------------------------------------------
bool g_gdbZ80 = false; // set from main.cpp via --gdb-arch=z80

#define NUM_Z80_REGS 13

static UINT16 z80_read(State8085 *state, int r) {
    UINT8 f = pack_flags(state->cc);
    switch (r) {
    case 0:
        return (UINT16)((state->a << 8) | f); // af (A high, F low)
    case 1:
        return (UINT16)((state->b << 8) | state->c); // bc
    case 2:
        return (UINT16)((state->d << 8) | state->e); // de
    case 3:
        return (UINT16)((state->h << 8) | state->l); // hl
    case 4:
        return state->sp;
    case 5:
        return state->pc;
    default:
        return 0; // ix, iy, af', bc', de', hl', ir
    }
}

static void z80_write(State8085 *state, int r, UINT16 v) {
    switch (r) {
    case 0:
        state->a = (v >> 8) & 0xFF;
        unpack_flags(state, v & 0xFF);
        break; // af
    case 1:
        state->b = (v >> 8) & 0xFF;
        state->c = v & 0xFF;
        break; // bc
    case 2:
        state->d = (v >> 8) & 0xFF;
        state->e = v & 0xFF;
        break; // de
    case 3:
        state->h = (v >> 8) & 0xFF;
        state->l = v & 0xFF;
        break; // hl
    case 4:
        state->sp = v;
        break;
    case 5:
        state->pc = v;
        break;
    default:
        break; // ix, iy, shadows, ir: ignored
    }
}

// Mode-aware register accessors used by the g/G/p/P packet handlers.
static int reg_count() { return g_gdbZ80 ? NUM_Z80_REGS : NUM_REGS; }
static int reg_bytes(int i) { return g_gdbZ80 ? 2 : reg_size(i); }
static UINT16 reg_get(State8085 *state, int i) {
    return g_gdbZ80 ? z80_read(state, i) : read_reg(state, i);
}
static void reg_set(State8085 *state, int i, UINT16 v) {
    if (g_gdbZ80)
        z80_write(state, i, v);
    else
        write_reg(state, i, v);
}

// z80 target description: just name the architecture; GDB supplies the
// (built-in) z80 register layout, which reg_get/reg_set match.
static const char *targetXMLz80 = "<?xml version=\"1.0\"?>\n"
                                  "<!DOCTYPE target SYSTEM \"gdb-target.dtd\">\n"
                                  "<target version=\"1.0\">\n"
                                  "  <architecture>z80</architecture>\n"
                                  "</target>\n";

// Serialize a register to hex (little-endian for 16-bit).
// Returns number of hex chars written.
static int hex_reg(char *buf, State8085 *state, int regnum) {
    UINT16 val = reg_get(state, regnum);
    int sz = reg_bytes(regnum);
    if (sz == 1) {
        hex_byte(buf, val & 0xFF);
        return 2;
    } else if (sz == 2) {
        // Little-endian: low byte first
        hex_byte(buf, val & 0xFF);
        hex_byte(buf + 2, (val >> 8) & 0xFF);
        return 4;
    }
    return 0;
}

//----------------------------------------------------------------------------
// GDB main loop
//----------------------------------------------------------------------------

int gdb_main(int port, State8085 *state, ExecutionStats8085 *stats, std::vector<PeriodicTimer> &timers) {
#ifdef _WIN32
    WinsockInit winsockInit;
#else
    signal(SIGPIPE, SIG_IGN);
#endif

    // Create listening socket
    socket_t listenFd = socket(AF_INET, SOCK_STREAM, 0);
    if (listenFd == INVALID_SOCKET) {
        perror("gdb: socket");
        return 1;
    }

    int opt = 1;
    setsockopt(listenFd, SOL_SOCKET, SO_REUSEADDR, (const char *)&opt, sizeof(opt));

    struct sockaddr_in addr;
    memset(&addr, 0, sizeof(addr));
    addr.sin_family = AF_INET;
    addr.sin_addr.s_addr = htonl(INADDR_LOOPBACK);
    addr.sin_port = htons(port);

    if (bind(listenFd, (struct sockaddr *)&addr, sizeof(addr)) < 0) {
        perror("gdb: bind");
        sock_close(listenFd);
        return 1;
    }

    if (listen(listenFd, 1) < 0) {
        perror("gdb: listen");
        sock_close(listenFd);
        return 1;
    }

    fprintf(stderr, "GDB stub listening on localhost:%d\n", port);
    fprintf(stderr, "Connect with: target remote localhost:%d\n", port);

    socket_t clientFd = accept(listenFd, nullptr, nullptr);
    if (clientFd == INVALID_SOCKET) {
        perror("gdb: accept");
        sock_close(listenFd);
        return 1;
    }
    sock_close(listenFd);
    fprintf(stderr, "GDB client connected.\n");

    std::set<UINT16> breakpoints;
    bool running = true;
    UINT64 gdbStep = 0;

    while (running) {
        std::string pkt = recv_packet(clientFd);
        if (pkt.empty()) {
            fprintf(stderr, "GDB disconnected.\n");
            break;
        }

        std::string reply;

        //--- Async break (Ctrl-C) ---
        if (pkt[0] == 0x03) {
            reply = "S05";
        }
        //--- Halt reason ---
        else if (pkt[0] == '?') {
            reply = "S05";
        }
        //--- Read all registers ---
        else if (pkt[0] == 'g') {
            // Total hex chars: 8 * 2 (8-bit) + 1 * 2 (FLAGS) + 4 * 4 (16-bit) = 34
            // Actually: B,C,D,E,H,L,M,A = 8*2=16; SP=4; FLAGS=2; PSW=4; PC=4; BC=4; DE=4; HL=4
            // Total = 16 + 4 + 2 + 4 + 4 + 4 + 4 + 4 = 42
            char buf[64];
            int pos = 0;
            for (int i = 0; i < reg_count(); i++)
                pos += hex_reg(buf + pos, state, i);
            buf[pos] = '\0';
            reply = buf;
        }
        //--- Write all registers ---
        else if (pkt[0] == 'G') {
            const char *p = pkt.c_str() + 1;
            for (int i = 0; i < reg_count(); i++) {
                int sz = reg_bytes(i);
                if (sz == 1) {
                    // 2 hex chars
                    if (!p[0] || !p[1])
                        break;
                    int h0 = hex_val(p[0]), h1 = hex_val(p[1]);
                    if (h0 < 0 || h1 < 0)
                        break;
                    write_reg(state, i, (h0 << 4) | h1);
                    p += 2;
                } else if (sz == 2) {
                    // 4 hex chars, little-endian
                    if (!p[0] || !p[1] || !p[2] || !p[3])
                        break;
                    int h0 = hex_val(p[0]), h1 = hex_val(p[1]);
                    int h2 = hex_val(p[2]), h3 = hex_val(p[3]);
                    if (h0 < 0 || h1 < 0 || h2 < 0 || h3 < 0)
                        break;
                    UINT8 lo = (h0 << 4) | h1;
                    UINT8 hi = (h2 << 4) | h3;
                    write_reg(state, i, (hi << 8) | lo);
                    p += 4;
                }
            }
            reply = "OK";
        }
        //--- Read single register ---
        else if (pkt[0] == 'p') {
            int regnum = (int)parse_hex(pkt.c_str() + 1, nullptr);
            if (regnum >= 0 && regnum < reg_count()) {
                char buf[5];
                int n = hex_reg(buf, state, regnum);
                buf[n] = '\0';
                reply = buf;
            } else {
                reply = "E00";
            }
        }
        //--- Write single register ---
        else if (pkt[0] == 'P') {
            const char *eq = strchr(pkt.c_str(), '=');
            if (eq) {
                int regnum = (int)parse_hex(pkt.c_str() + 1, nullptr);
                const char *vp = eq + 1;
                int sz = reg_bytes(regnum);
                UINT16 val = 0;

                if (sz == 1) {
                    int h0 = hex_val(vp[0]), h1 = hex_val(vp[1]);
                    val = ((h0 >= 0 && h1 >= 0) ? ((h0 << 4) | h1) : 0) & 0xFF;
                } else if (sz == 2) {
                    // Little-endian: first byte is low
                    int h0 = hex_val(vp[0]), h1 = hex_val(vp[1]);
                    int h2 = hex_val(vp[2]), h3 = hex_val(vp[3]);
                    UINT8 lo = (h0 >= 0 && h1 >= 0) ? ((h0 << 4) | h1) : 0;
                    UINT8 hi = (h2 >= 0 && h3 >= 0) ? ((h2 << 4) | h3) : 0;
                    val = (hi << 8) | lo;
                }

                if (regnum >= 0 && regnum < reg_count()) {
                    reg_set(state, regnum, val);
                    reply = "OK";
                } else {
                    reply = "E00";
                }
            } else {
                reply = "E00";
            }
        }
        //--- Read memory ---
        else if (pkt[0] == 'm') {
            const char *comma = strchr(pkt.c_str(), ',');
            if (comma) {
                UINT16 maddr = (UINT16)parse_hex(pkt.c_str() + 1, nullptr);
                unsigned long len = parse_hex(comma + 1, nullptr);
                std::string hex;
                hex.reserve(len * 2);
                char hb[2];
                for (unsigned long i = 0; i < len; i++) {
                    hex_byte(hb, state->memory[(maddr + i) & 0xFFFF]);
                    hex += hb[0];
                    hex += hb[1];
                }
                reply = hex;
            } else {
                reply = "E00";
            }
        }
        //--- Write memory ---
        else if (pkt[0] == 'M') {
            const char *comma = strchr(pkt.c_str(), ',');
            const char *colon = strchr(pkt.c_str(), ':');
            if (comma && colon) {
                UINT16 maddr = (UINT16)parse_hex(pkt.c_str() + 1, nullptr);
                unsigned long len = parse_hex(comma + 1, nullptr);
                const char *hexdata = colon + 1;
                for (unsigned long i = 0; i < len && hexdata[0] && hexdata[1]; i++) {
                    int h0 = hex_val(hexdata[0]);
                    int h1 = hex_val(hexdata[1]);
                    if (h0 < 0 || h1 < 0)
                        break;
                    state->memory[(maddr + i) & 0xFFFF] = (h0 << 4) | h1;
                    hexdata += 2;
                }
                reply = "OK";
            } else {
                reply = "E00";
            }
        }
        //--- Single step ---
        else if (pkt[0] == 's') {
            if (pkt.size() > 1)
                state->pc = (UINT16)parse_hex(pkt.c_str() + 1, nullptr);

            fire_timers(state, stats, timers);
            io_runtime_on_step(gdbStep, stats->total_tstates);
            Emulate8085Op(state, stats);
            gdbStep++;
            fire_timers(state, stats, timers);

            reply = "S05";
        }
        //--- Continue ---
        else if (pkt[0] == 'c') {
            if (pkt.size() > 1)
                state->pc = (UINT16)parse_hex(pkt.c_str() + 1, nullptr);

            // Step once past current PC if it's a breakpoint
            if (breakpoints.count(state->pc)) {
                fire_timers(state, stats, timers);
                io_runtime_on_step(gdbStep, stats->total_tstates);
                Emulate8085Op(state, stats);
                gdbStep++;
                fire_timers(state, stats, timers);
            }

            int pollCounter = 0;
            for (;;) {
                fire_timers(state, stats, timers);
                io_runtime_on_step(gdbStep, stats->total_tstates);
                int halted = Emulate8085Op(state, stats);
                gdbStep++;
                fire_timers(state, stats, timers);

                if (breakpoints.count(state->pc)) {
                    reply = "S05";
                    break;
                }

                if (halted && timers.empty()) {
                    reply = "S05";
                    break;
                }

                if (++pollCounter >= 1024) {
                    pollCounter = 0;
                    if (sock_poll_readable(clientFd) > 0) {
                        char brk;
                        if (sock_recv(clientFd, &brk, 1) == 1 && brk == 0x03) {
                            reply = "S02";
                            break;
                        }
                    }
                }
            }
        }
        //--- Set/clear software breakpoint ---
        else if ((pkt[0] == 'Z' || pkt[0] == 'z') && pkt.size() >= 3 && pkt[1] == '0' && pkt[2] == ',') {
            UINT16 bpAddr = (UINT16)parse_hex(pkt.c_str() + 3, nullptr);
            if (pkt[0] == 'Z')
                breakpoints.insert(bpAddr);
            else
                breakpoints.erase(bpAddr);
            reply = "OK";
        }
        //--- qSupported ---
        else if (pkt.compare(0, 10, "qSupported") == 0) {
            reply = "PacketSize=4096;qXfer:features:read+";
        }
        //--- Target description XML ---
        else if (pkt.compare(0, 31, "qXfer:features:read:target.xml:") == 0) {
            const char *args = pkt.c_str() + 31;
            const char *comma = strchr(args, ',');
            if (comma) {
                unsigned long offset = parse_hex(args, nullptr);
                unsigned long length = parse_hex(comma + 1, nullptr);
                const char *xml = g_gdbZ80 ? targetXMLz80 : targetXML;
                size_t xmlLen = strlen(xml);

                if (offset >= xmlLen) {
                    reply = "l";
                } else {
                    size_t remaining = xmlLen - offset;
                    size_t chunk = remaining < length ? remaining : length;
                    bool last = (offset + chunk >= xmlLen);
                    reply = last ? "l" : "m";
                    reply += std::string(xml + offset, chunk);
                }
            } else {
                reply = "E00";
            }
        }
        //--- Thread queries (single-thread stubs) ---
        else if (pkt == "qfThreadInfo") {
            reply = "m1";
        } else if (pkt == "qsThreadInfo") {
            reply = "l";
        } else if (pkt == "qC") {
            reply = "QC1";
        } else if (pkt == "qAttached") {
            reply = "1";
        }
        //--- Thread selection (always OK) ---
        else if (pkt[0] == 'H') {
            reply = "OK";
        }
        //--- Kill ---
        else if (pkt[0] == 'k') {
            fprintf(stderr, "GDB kill request.\n");
            sock_close(clientFd);
            return 0;
        }
        //--- Detach ---
        else if (pkt[0] == 'D') {
            fprintf(stderr, "GDB detached.\n");
            send_packet(clientFd, "OK");
            sock_close(clientFd);
            return 0;
        }
        //--- Unsupported packet ---
        else {
            reply = "";
        }

        send_packet(clientFd, reply);
    }

    sock_close(clientFd);
    return 0;
}
