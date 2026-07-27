//----------------------------------------------------------------------------
// mc6850_28c256.cpp - i8085-trace I/O plugin for HEX-loader tests
//
// Emulates:
//   * an MC68B50 (6850) ACIA on ports 0xDE (status R / control W) and
//     0xDF (data R/W), with a 1-byte receive holding register, scripted RX,
//     TX capture, RST 7.5 on receive (when RX interrupt enabled), and /RTS
//     flow control (host stops sending while /RTS is de-asserted).
//   * a 28C256 EEPROM in [0x0000,0x8000): when wren=off every CPU write to
//     that range is reverted (so /DATA polling never verifies -> the loader
//     detects write-protect); when wren=on writes are accepted.
//
// Config string (';'-separated key=value):
//   rx=<hexbytes>      bytes to feed the CPU (e.g. "41 42 0D" or "41420D")
//   rxfile=<path>      binary file of RX bytes (alternative to rx=)
//   txlog=<path>       append every transmitted byte here (binary)
//   txlog=-            stream transmitted bytes live to stdout ("stdout" also)
//   rtslog=<path>      append "/RTS" transitions here (text; else stderr)
//   wren=on|off        EEPROM write enable (default on)
//   bytegap=<n>        min instruction steps between received bytes (default 64)
//   tcp=<port>         bridge the ACIA to a raw TCP socket on localhost:<port>
//                      for a live interactive terminal (pyserial socket://)
//   gdbtx=1            mirror transmitted bytes to stdout as GDB-safe, CR-free,
//                      LF-terminated lines. For debug sessions: PlatformIO
//                      forwards the server's stdout into the GDB/MI stream, so
//                      the UART then appears in the debug console. Raw stdout
//                      (txlog=-) would corrupt that stream, so use gdbtx there.
//----------------------------------------------------------------------------

#include "i8085_io_plugin.h"

#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <deque>
#include <string>
#include <vector>

#if defined(_WIN32)
#include <winsock2.h>
#include <ws2tcpip.h>
#include <fcntl.h> // _O_BINARY
#include <io.h>    // _setmode, _fileno
typedef SOCKET sock_t;
static const sock_t BAD_SOCK = INVALID_SOCKET;
static bool sock_would_block() { return WSAGetLastError() == WSAEWOULDBLOCK; }
static void sock_close_one(sock_t s) { closesocket(s); }
static void sock_nonblock(sock_t s) { u_long m = 1; ioctlsocket(s, FIONBIO, &m); }
#define PLUGIN_EXPORT extern "C" __declspec(dllexport)
#else
#include <arpa/inet.h>
#include <errno.h>
#include <fcntl.h>
#include <netinet/in.h>
#include <sys/socket.h>
#include <unistd.h>
typedef int sock_t;
static const sock_t BAD_SOCK = -1;
static bool sock_would_block() { return errno == EWOULDBLOCK || errno == EAGAIN; }
static void sock_close_one(sock_t s) { close(s); }
static void sock_nonblock(sock_t s) { int f = fcntl(s, F_GETFL, 0); fcntl(s, F_SETFL, f | O_NONBLOCK); }
#define PLUGIN_EXPORT extern "C" __attribute__((visibility("default")))
#endif

static const UINT8 ACIA_CTRL = 0xDE; // status (R) / control (W)
static const UINT8 ACIA_DATA = 0xDF; // rx (R) / tx (W)
static const UINT8 EEPROM_END = 0x80; // high byte: [0x0000,0x8000)

struct Ctx {
    std::vector<UINT8> rx;
    size_t rxPos = 0;
    bool rdrFull = false;
    UINT8 rdrByte = 0;
    UINT64 nextArrival = 0;
    UINT64 bytegap = 64;
    UINT64 lastStep = 0;

    FILE *txlog = nullptr;
    bool txIsStdout = false; // txlog aliases stdout; do not fclose it
    FILE *rtslog = nullptr;

    bool rxIntEnabled = false;
    bool rtsAsserted = true; // ready to receive by default

    bool wrenOn = true;
    std::vector<UINT8> shadow; // snapshot of [0,0x8000)
    bool shadowValid = false;

    // Optional live TCP bridge (tcp=PORT): the ACIA <-> a raw socket, so a
    // serial monitor (pyserial socket://) gives an interactive terminal.
    bool tcpMode = false;
    sock_t listenSock = BAD_SOCK;
    sock_t clientSock = BAD_SOCK;
    std::deque<UINT8> rxq;  // bytes received from the socket, feeding the RDR
    std::deque<UINT8> txbuf; // TX emitted before a client connected (banner)

    // Optional GDB-safe TX mirror (gdbtx=1): duplicate transmitted bytes to
    // stdout as whole, CR-stripped, LF-terminated lines. PlatformIO wraps a
    // debug server's stdout into GDB/MI `@"..."` stream records, but its escaper
    // does not escape CR and only terminates a record when the chunk ends in LF
    // -- so raw byte-by-byte UART would corrupt the MI stream (mangling the
    // following *stopped record). Emitting clean lines keeps the mirror safe.
    bool gdbtx = false;
    std::string gdbline;
    UINT64 gdbLastTxStep = 0;
};

static std::string cfgValue(const char *config, const char *key) {
    if (!config)
        return "";
    std::string s(config);
    std::string k = std::string(key) + "=";
    size_t p = 0;
    while (p < s.size()) {
        size_t semi = s.find(';', p);
        std::string tok = s.substr(p, semi == std::string::npos ? std::string::npos : semi - p);
        if (tok.rfind(k, 0) == 0)
            return tok.substr(k.size());
        if (semi == std::string::npos)
            break;
        p = semi + 1;
    }
    return "";
}

static void parseHexBytes(const std::string &in, std::vector<UINT8> &out) {
    std::string h;
    for (char c : in)
        if (isxdigit((unsigned char)c))
            h.push_back(c);
    for (size_t i = 0; i + 1 < h.size(); i += 2)
        out.push_back((UINT8)strtol(h.substr(i, 2).c_str(), nullptr, 16));
}

static void logRts(Ctx *c, bool asserted, UINT64 step) {
    FILE *f = c->rtslog ? c->rtslog : stderr;
    fprintf(f, "RTS=%d step=%llu\n", asserted ? 1 : 0, (unsigned long long)step);
    if (c->rtslog)
        fflush(c->rtslog);
}

static void gdb_tx_flush(Ctx *c); // defined below

static void destroy(void *vctx) {
    Ctx *c = (Ctx *)vctx;
    if (!c)
        return;
    if (c->gdbtx && !c->gdbline.empty())
        gdb_tx_flush(c);
    if (c->txlog && !c->txIsStdout)
        fclose(c->txlog);
    if (c->rtslog)
        fclose(c->rtslog);
    if (c->clientSock != BAD_SOCK)
        sock_close_one(c->clientSock);
    if (c->listenSock != BAD_SOCK)
        sock_close_one(c->listenSock);
    delete c;
}

static void on_reset(void *vctx, State8085 *state) {
    Ctx *c = (Ctx *)vctx;
    c->rxPos = 0;
    c->rdrFull = false;
    c->nextArrival = 0;
    c->rxIntEnabled = false;
    c->rtsAsserted = true;
    c->shadowValid = false;
    (void)state;
}

static void snapshot(Ctx *c, State8085 *state) {
    c->shadow.assign(state->memory, state->memory + (EEPROM_END << 8));
    c->shadowValid = true;
}

static void on_step(void *vctx, State8085 *state, UINT64 step, UINT64 tstates) {
    Ctx *c = (Ctx *)vctx;
    (void)tstates;
    c->lastStep = step;

    // EEPROM / WREN model: reconcile [0,0x8000) with the shadow.
    if (!c->shadowValid)
        snapshot(c, state);
    else if (memcmp(state->memory, c->shadow.data(), (EEPROM_END << 8)) != 0) {
        for (size_t i = 0; i < (size_t)(EEPROM_END << 8); i++) {
            if (state->memory[i] != c->shadow[i]) {
                if (c->wrenOn)
                    c->shadow[i] = state->memory[i]; // accept write
                else
                    state->memory[i] = c->shadow[i]; // revert (write-protected)
            }
        }
    }

    // Live TCP bridge: accept a client and drain any received bytes into the
    // RX queue; those feed the RDR below just like scripted input.
    if (c->tcpMode) {
        if (c->clientSock == BAD_SOCK) {
            sock_t cs = accept(c->listenSock, nullptr, nullptr);
            if (cs != BAD_SOCK) {
                sock_nonblock(cs);
                c->clientSock = cs;
                // Flush any TX buffered before the client connected (e.g. the
                // startup banner) so it is not lost.
                while (!c->txbuf.empty()) {
                    char b = (char)c->txbuf.front();
                    c->txbuf.pop_front();
                    send(cs, &b, 1, 0);
                }
            }
        }
        if (c->clientSock != BAD_SOCK) {
            unsigned char b[64];
            int n = (int)recv(c->clientSock, (char *)b, sizeof b, 0);
            if (n > 0) {
                for (int i = 0; i < n; i++)
                    c->rxq.push_back(b[i]);
            } else if (n == 0) { // client disconnected
                sock_close_one(c->clientSock);
                c->clientSock = BAD_SOCK;
            }
        }
    }

    // Peek the next scripted byte into the receive holding register. rxPos is
    // only advanced when the byte is actually consumed (read of 0xDF), so a
    // 68B50 master reset that clears an unread byte returns it to the queue.
    if (!c->rdrFull && c->rxPos < c->rx.size() && c->rtsAsserted && step >= c->nextArrival) {
        c->rdrByte = c->rx[c->rxPos];
        c->rdrFull = true;
    }

    // Feed a live (TCP) byte into the RDR when free. No bytegap pacing: live
    // input arrives as fast as it is typed.
    if (!c->rdrFull && !c->rxq.empty() && c->rtsAsserted) {
        c->rdrByte = c->rxq.front();
        c->rxq.pop_front();
        c->rdrFull = true;
    }

    // The 6850 /IRQ is level-based on (RDRF & RX-int-enable); assert the 8085's
    // RST 7.5 latch while a byte is pending and RX interrupts are enabled, so a
    // byte that arrived before EI still interrupts once interrupts come on.
    if (c->rdrFull && c->rxIntEnabled)
        state->r7_latch = 1;

    // Flush a pending GDB-mirror line once transmission goes idle (e.g. a "> "
    // prompt with no trailing LF) so it shows promptly in the debug console.
    if (c->gdbtx && !c->gdbline.empty() && step - c->gdbLastTxStep >= 256)
        gdb_tx_flush(c);
}

static void on_io_pre_read(void *vctx, State8085 *state, UINT8 port) {
    Ctx *c = (Ctx *)vctx;
    if (port == ACIA_CTRL)
        state->io[ACIA_CTRL] = 0x02 | (c->rdrFull ? 0x01 : 0x00); // TDRE | RDRF
    else if (port == ACIA_DATA)
        state->io[ACIA_DATA] = c->rdrByte;
}

static void on_io_post_read(void *vctx, State8085 *state, UINT8 port, UINT8 value) {
    Ctx *c = (Ctx *)vctx;
    (void)state;
    (void)value;
    if (port == ACIA_DATA && c->rdrFull) {
        c->rdrFull = false;
        c->rxPos++;                                // consumed: advance the queue
        c->nextArrival = c->lastStep + c->bytegap; // pace the next received byte
    }
}

// Emit the buffered GDB-mirror line + a real LF to stdout, so PlatformIO's MI
// stream escaper produces a single, properly terminated `@"..."` record.
static void gdb_tx_flush(Ctx *c) {
    if (!c->gdbline.empty())
        fwrite(c->gdbline.data(), 1, c->gdbline.size(), stdout);
    fputc('\n', stdout);
    fflush(stdout);
    c->gdbline.clear();
}

// Accumulate one transmitted byte into the GDB-mirror line buffer. CR is
// dropped (PlatformIO does not escape it); LF flushes the line; other control
// or non-ASCII bytes are dropped so they cannot break MI record framing.
static void gdb_tx_byte(Ctx *c, UINT8 v) {
    if (v == '\n')
        gdb_tx_flush(c);
    else if (v == '\r')
        return;
    else if (v == '\t' || (v >= 0x20 && v <= 0x7E)) {
        c->gdbline.push_back((char)v);
        if (c->gdbline.size() >= 512) // cap unusually long lines
            gdb_tx_flush(c);
    }
}

static void on_io_write(void *vctx, State8085 *state, UINT8 port, UINT8 value) {
    Ctx *c = (Ctx *)vctx;
    (void)state;
    if (port == ACIA_DATA) {
        if (c->txlog) {
            fputc(value, c->txlog);
            fflush(c->txlog);
        }
        if (c->tcpMode) {
            if (c->clientSock != BAD_SOCK) {
                char b = (char)value;
                if (send(c->clientSock, &b, 1, 0) < 0 && !sock_would_block()) {
                    sock_close_one(c->clientSock);
                    c->clientSock = BAD_SOCK;
                }
            } else if (c->txbuf.size() < 8192) {
                // No client yet: buffer (bounded) so the banner survives.
                c->txbuf.push_back(value);
            }
        }
        if (c->gdbtx) {
            gdb_tx_byte(c, value);
            c->gdbLastTxStep = c->lastStep;
        }
        return;
    }
    if (port == ACIA_CTRL) {
        if ((value & 0x03) == 0x03) { // master reset: /RTS held high (de-asserted)
            c->rdrFull = false;
            c->rxIntEnabled = false;
            if (c->rtsAsserted) {
                c->rtsAsserted = false;
                logRts(c, false, c->lastStep);
            }
            return;
        }
        c->rxIntEnabled = (value & 0x80) != 0;
        bool newRts = (value & 0x60) != 0x40; // transmit-control bits == 10 -> /RTS high (stop)
        if (newRts != c->rtsAsserted) {
            c->rtsAsserted = newRts;
            logRts(c, newRts, c->lastStep);
        }
    }
}

PLUGIN_EXPORT int i8085_io_plugin_init(const char *config, void **out_ctx, I8085IoPluginAPI *out_api, char *errbuf,
                                       size_t errbuf_len) {
    Ctx *c = new Ctx();

    std::string rxfile = cfgValue(config, "rxfile");
    if (!rxfile.empty()) {
        FILE *f = fopen(rxfile.c_str(), "rb");
        if (!f) {
            snprintf(errbuf, errbuf_len, "cannot open rxfile %s", rxfile.c_str());
            delete c;
            return -1;
        }
        int ch;
        while ((ch = fgetc(f)) != EOF)
            c->rx.push_back((UINT8)ch);
        fclose(f);
    } else {
        parseHexBytes(cfgValue(config, "rx"), c->rx);
    }

    std::string tx = cfgValue(config, "txlog");
    if (tx == "-" || tx == "stdout") {
        // Stream transmitted bytes live to stdout (no file).
        c->txlog = stdout;
        c->txIsStdout = true;
    } else if (!tx.empty()) {
        c->txlog = fopen(tx.c_str(), "wb");
    }

    std::string rl = cfgValue(config, "rtslog");
    if (!rl.empty())
        c->rtslog = fopen(rl.c_str(), "w");

    std::string wren = cfgValue(config, "wren");
    c->wrenOn = !(wren == "off" || wren == "0" || wren == "false");

    std::string bg = cfgValue(config, "bytegap");
    if (!bg.empty())
        c->bytegap = strtoull(bg.c_str(), nullptr, 10);

    // gdbtx=1: mirror transmitted bytes to stdout as GDB-safe lines. Intended
    // for debug sessions, where PlatformIO forwards the server's stdout into the
    // GDB/MI stream so the UART shows up in the debug console.
    std::string gx = cfgValue(config, "gdbtx");
    c->gdbtx = (gx == "1" || gx == "on" || gx == "true" || gx == "yes");
    if (c->gdbtx) {
        // Emit LF-only line terminators: on Windows the default text-mode CRT
        // would translate our '\n' back into '\r\n', re-inserting the CR we just
        // stripped (and PlatformIO's MI escaper does not escape CR). Force the
        // stdout stream to binary so the mirror stays CR-free.
#if defined(_WIN32)
        _setmode(_fileno(stdout), _O_BINARY);
#endif
    }

    // Live TCP bridge: tcp=PORT listens on localhost:PORT and bridges the ACIA
    // to a raw socket, so a serial monitor (e.g. socket://localhost:PORT) is an
    // interactive terminal.
    std::string tcp = cfgValue(config, "tcp");
    if (!tcp.empty()) {
        int port = atoi(tcp.c_str());
#if defined(_WIN32)
        WSADATA wsa;
        WSAStartup(MAKEWORD(2, 2), &wsa);
#endif
        sock_t ls = socket(AF_INET, SOCK_STREAM, 0);
        if (ls != BAD_SOCK) {
            int yes = 1;
            setsockopt(ls, SOL_SOCKET, SO_REUSEADDR, (const char *)&yes, sizeof yes);
            struct sockaddr_in a;
            memset(&a, 0, sizeof a);
            a.sin_family = AF_INET;
            a.sin_addr.s_addr = htonl(INADDR_LOOPBACK);
            a.sin_port = htons((unsigned short)port);
            if (bind(ls, (struct sockaddr *)&a, sizeof a) == 0 && listen(ls, 1) == 0) {
                sock_nonblock(ls);
                c->listenSock = ls;
                c->tcpMode = true;
                fprintf(stderr,
                        "[mc6850] ACIA bridged to TCP localhost:%d "
                        "(connect a serial monitor via socket://localhost:%d)\n",
                        port, port);
            } else {
                fprintf(stderr, "[mc6850] tcp: bind/listen on port %d failed\n", port);
                sock_close_one(ls);
            }
        }
    }

    out_api->abi_version = I8085_IO_PLUGIN_ABI_VERSION;
    out_api->destroy = destroy;
    out_api->on_reset = on_reset;
    out_api->on_step = on_step;
    out_api->on_io_write = on_io_write;
    out_api->on_io_pre_read = on_io_pre_read;
    out_api->on_io_post_read = on_io_post_read;
    *out_ctx = c;
    (void)errbuf_len;
    return 0;
}
