//----------------------------------------------------------------------------
// io_channel.cpp - core byte-channel services (TCP / window / stdout / gdbtx).
//
// Implements the I8085HostAPI handed to plugins at init. All platform-specific
// networking and terminal spawning lives here, behind channel_open/read/write/
// close, so plugins stay portable.
//----------------------------------------------------------------------------

#include "io_channel.h"

#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <deque>
#include <string>
#include <vector>

#if defined(_WIN32)
#include <winsock2.h>
#include <ws2tcpip.h>
#include <windows.h>
#include <fcntl.h>
#include <io.h>
typedef SOCKET sock_t;
static const sock_t BAD_SOCK = INVALID_SOCKET;
static bool sock_would_block() { return WSAGetLastError() == WSAEWOULDBLOCK; }
static void sock_close_one(sock_t s) { closesocket(s); }
static void sock_nonblock(sock_t s) { u_long m = 1; ioctlsocket(s, FIONBIO, &m); }
#else
#include <arpa/inet.h>
#include <errno.h>
#include <netinet/in.h>
#include <sys/socket.h>
#include <sys/types.h>
#include <fcntl.h>
#include <unistd.h>
typedef int sock_t;
static const sock_t BAD_SOCK = -1;
static bool sock_would_block() { return errno == EWOULDBLOCK || errno == EAGAIN; }
static void sock_close_one(sock_t s) { close(s); }
static void sock_nonblock(sock_t s) { int f = fcntl(s, F_GETFL, 0); fcntl(s, F_SETFL, f | O_NONBLOCK); }
#endif

//----------------------------------------------------------------------------

namespace {

std::string gExePath;

void ensureWinsock() {
#if defined(_WIN32)
    static bool done = false;
    if (!done) {
        WSADATA wsa;
        WSAStartup(MAKEWORD(2, 2), &wsa);
        done = true;
    }
#endif
}

// Path to the bundled i8085-console helper (next to the simulator exe, or the
// I8085_CONSOLE override).
std::string consoleHelperPath() {
    if (const char *ov = getenv("I8085_CONSOLE"))
        return ov;
    std::string exe = gExePath;
#if defined(_WIN32)
    char buf[MAX_PATH];
    if (GetModuleFileNameA(nullptr, buf, sizeof buf))
        exe = buf;
#endif
    size_t slash = exe.find_last_of("/\\");
    std::string dir = (slash == std::string::npos) ? "." : exe.substr(0, slash);
#if defined(_WIN32)
    return dir + "\\i8085-console.exe";
#else
    return dir + "/i8085-console";
#endif
}

// Spawn a terminal window running the console helper against localhost:port.
// Best-effort; failures are logged and non-fatal (the tcp bridge still works).
void spawnConsoleWindow(int port) {
    std::string helper = consoleHelperPath();
    std::string portStr = std::to_string(port);
#if defined(_WIN32)
    // `start "title" "exe" args` opens a new console window.
    std::string cmd = "cmd /c start \"i8085 UART " + portStr + "\" \"" + helper + "\" 127.0.0.1 " + portStr;
    if (system(cmd.c_str()) != 0)
        fprintf(stderr, "[io_channel] could not spawn console window for port %d\n", port);
#else
    std::string inner = "\"" + helper + "\" 127.0.0.1 " + portStr;
    pid_t pid = fork();
    if (pid == 0) {
#if defined(__APPLE__)
        std::string osa = "tell application \"Terminal\" to do script \"" + inner + "\"";
        execlp("osascript", "osascript", "-e", osa.c_str(), (char *)nullptr);
#else
        execlp("x-terminal-emulator", "x-terminal-emulator", "-e", "sh", "-c", inner.c_str(), (char *)nullptr);
        execlp("xterm", "xterm", "-e", "sh", "-c", inner.c_str(), (char *)nullptr);
#endif
        _exit(127);
    } else if (pid < 0) {
        fprintf(stderr, "[io_channel] could not fork console window for port %d\n", port);
    }
#endif
}

//----------------------------------------------------------------------------

struct Channel {
    virtual ~Channel() {}
    virtual int read(void *, int) { return 0; }
    virtual int write(const void *buf, int len) { return len; }
    virtual void tick(UINT64) {}
};

// TCP-bridged serial channel: listens on localhost:port, bridges one client at
// a time. Optionally spawns an i8085-console terminal window bound to it.
struct TcpChannel : Channel {
    sock_t listenSock = BAD_SOCK;
    sock_t clientSock = BAD_SOCK;
    std::deque<unsigned char> rxq;
    std::deque<unsigned char> txbuf; // buffered until a client connects (banner)

    bool open(int port, bool spawnWindow, char *err, size_t errlen) {
        ensureWinsock();
        sock_t ls = socket(AF_INET, SOCK_STREAM, 0);
        if (ls == BAD_SOCK) {
            snprintf(err, errlen, "socket() failed");
            return false;
        }
        int yes = 1;
        setsockopt(ls, SOL_SOCKET, SO_REUSEADDR, (const char *)&yes, sizeof yes);
        struct sockaddr_in a;
        memset(&a, 0, sizeof a);
        a.sin_family = AF_INET;
        a.sin_addr.s_addr = htonl(INADDR_LOOPBACK);
        a.sin_port = htons((unsigned short)port);
        if (bind(ls, (struct sockaddr *)&a, sizeof a) != 0 || listen(ls, 1) != 0) {
            snprintf(err, errlen, "bind/listen on port %d failed", port);
            sock_close_one(ls);
            return false;
        }
        // Learn the actually-bound port (matters when port==0 auto-assign).
        struct sockaddr_in bound;
        socklen_t blen = sizeof bound;
        if (getsockname(ls, (struct sockaddr *)&bound, &blen) == 0)
            port = ntohs(bound.sin_port);
        sock_nonblock(ls);
        listenSock = ls;
        fprintf(stderr, "[io_channel] serial bridged to TCP localhost:%d%s\n", port,
                spawnWindow ? " (opening console window)" : " (connect via socket://localhost:PORT)");
        if (spawnWindow)
            spawnConsoleWindow(port);
        return true;
    }

    ~TcpChannel() override {
        if (clientSock != BAD_SOCK)
            sock_close_one(clientSock);
        if (listenSock != BAD_SOCK)
            sock_close_one(listenSock);
    }

    void tick(UINT64) override {
        if (clientSock == BAD_SOCK && listenSock != BAD_SOCK) {
            sock_t cs = accept(listenSock, nullptr, nullptr);
            if (cs != BAD_SOCK) {
                sock_nonblock(cs);
                clientSock = cs;
                while (!txbuf.empty()) {
                    char b = (char)txbuf.front();
                    txbuf.pop_front();
                    send(cs, &b, 1, 0);
                }
            }
        }
        if (clientSock != BAD_SOCK) {
            unsigned char b[64];
            int n = (int)recv(clientSock, (char *)b, sizeof b, 0);
            if (n > 0) {
                for (int i = 0; i < n; i++)
                    rxq.push_back(b[i]);
            } else if (n == 0) {
                sock_close_one(clientSock);
                clientSock = BAD_SOCK;
            }
        }
    }

    int read(void *buf, int maxlen) override {
        int n = 0;
        unsigned char *out = (unsigned char *)buf;
        while (n < maxlen && !rxq.empty()) {
            out[n++] = rxq.front();
            rxq.pop_front();
        }
        return n;
    }

    int write(const void *buf, int len) override {
        const char *p = (const char *)buf;
        if (clientSock != BAD_SOCK) {
            if (send(clientSock, p, len, 0) < 0 && !sock_would_block()) {
                sock_close_one(clientSock);
                clientSock = BAD_SOCK;
            }
        } else {
            for (int i = 0; i < len && txbuf.size() < 8192; i++)
                txbuf.push_back((unsigned char)p[i]);
        }
        return len;
    }
};

// Raw write-only mirror to the host stdout (e.g. live-streaming for `pio run`).
struct StdoutChannel : Channel {
    int write(const void *buf, int len) override {
        fwrite(buf, 1, (size_t)len, stdout);
        fflush(stdout);
        return len;
    }
};

// GDB/MI-safe write-only mirror: buffers a line, drops CR, drops non-printables,
// and flushes a whole LF-terminated line so PlatformIO's `@"..."` stream escaper
// produces one well-formed record. Idle output (a prompt with no LF) is flushed
// after a short quiet period so it still appears promptly.
struct GdbtxChannel : Channel {
    std::string line;
    int idleTicks = 0;

    GdbtxChannel() {
#if defined(_WIN32)
        // Force binary stdout so our LF is not translated back to CRLF (which
        // would re-insert the CR that PlatformIO's escaper does not escape).
        _setmode(_fileno(stdout), _O_BINARY);
#endif
    }
    ~GdbtxChannel() override {
        if (!line.empty())
            flush();
    }

    void flush() {
        if (!line.empty())
            fwrite(line.data(), 1, line.size(), stdout);
        fputc('\n', stdout);
        fflush(stdout);
        line.clear();
        idleTicks = 0;
    }

    int write(const void *buf, int len) override {
        const unsigned char *p = (const unsigned char *)buf;
        for (int i = 0; i < len; i++) {
            unsigned char v = p[i];
            idleTicks = 0;
            if (v == '\n')
                flush();
            else if (v == '\r')
                continue;
            else if (v == '\t' || (v >= 0x20 && v <= 0x7E)) {
                line.push_back((char)v);
                if (line.size() >= 512)
                    flush();
            }
        }
        return len;
    }

    void tick(UINT64) override {
        if (!line.empty() && ++idleTicks >= 256)
            flush();
    }
};

std::vector<Channel *> gChannels;

//----------------------------------------------------------------------------
// I8085HostAPI implementation

void *host_channel_open(const char *spec, char *errbuf, size_t errbuf_len) {
    if (errbuf && errbuf_len)
        errbuf[0] = '\0';
    std::string s = spec ? spec : "";
    std::string type = s;
    std::string arg;
    size_t eq = s.find('=');
    if (eq != std::string::npos) {
        type = s.substr(0, eq);
        arg = s.substr(eq + 1);
    }

    Channel *ch = nullptr;
    if (type == "tcp" || type == "window") {
        int port = arg.empty() ? 0 : atoi(arg.c_str());
        TcpChannel *t = new TcpChannel();
        char e[128] = {0};
        if (!t->open(port, type == "window", e, sizeof e)) {
            if (errbuf && errbuf_len)
                snprintf(errbuf, errbuf_len, "%s", e);
            delete t;
            return nullptr;
        }
        ch = t;
    } else if (type == "stdout") {
        ch = new StdoutChannel();
    } else if (type == "gdbtx") {
        ch = new GdbtxChannel();
    } else {
        if (errbuf && errbuf_len)
            snprintf(errbuf, errbuf_len, "unknown channel spec '%s'", spec ? spec : "");
        return nullptr;
    }
    gChannels.push_back(ch);
    return ch;
}

int host_channel_read(void *chan, void *buf, int maxlen) {
    if (!chan || maxlen <= 0)
        return 0;
    return ((Channel *)chan)->read(buf, maxlen);
}

int host_channel_write(void *chan, const void *buf, int len) {
    if (!chan || len <= 0)
        return 0;
    return ((Channel *)chan)->write(buf, len);
}

void host_channel_close(void *chan) {
    if (!chan)
        return;
    for (auto it = gChannels.begin(); it != gChannels.end(); ++it) {
        if (*it == chan) {
            gChannels.erase(it);
            break;
        }
    }
    delete (Channel *)chan;
}

const I8085HostAPI gHostApi = {
    I8085_HOST_API_VERSION,
    host_channel_open,
    host_channel_read,
    host_channel_write,
    host_channel_close,
};

} // namespace

//----------------------------------------------------------------------------

extern "C" const I8085HostAPI *io_channels_host_api(void) {
    return &gHostApi;
}

extern "C" void io_channels_set_exe_path(const char *argv0) {
    if (argv0)
        gExePath = argv0;
}

extern "C" void io_channels_tick(UINT64 step) {
    for (Channel *c : gChannels)
        c->tick(step);
}

extern "C" void io_channels_shutdown(void) {
    for (Channel *c : gChannels)
        delete c;
    gChannels.clear();
}
