//----------------------------------------------------------------------------
// i8085-console - tiny raw-mode terminal client for an i8085-trace UART.
//
// Connects to a TCP port that the MC6850 ACIA plugin bridges (tcp=PORT), puts
// the terminal into raw mode, and shuttles bytes both ways: socket -> stdout,
// stdin -> socket. The i8085-trace ACIA plugin auto-spawns one of these in a
// fresh console window per UART marked window=1, giving each UART its own
// interactive terminal. The emulated target echoes typed characters, so this
// client does NOT local-echo.
//
// Usage: i8085-console <host> <port>
//----------------------------------------------------------------------------

#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <thread>

#if defined(_WIN32)
#include <winsock2.h>
#include <ws2tcpip.h>
#include <windows.h>
#include <io.h>
#include <fcntl.h>
typedef SOCKET sock_t;
static const sock_t BAD_SOCK = INVALID_SOCKET;
#else
#include <arpa/inet.h>
#include <netdb.h>
#include <netinet/in.h>
#include <sys/socket.h>
#include <termios.h>
#include <unistd.h>
typedef int sock_t;
static const sock_t BAD_SOCK = -1;
#endif

static sock_t connectTo(const char *host, const char *port) {
#if defined(_WIN32)
    WSADATA wsa;
    WSAStartup(MAKEWORD(2, 2), &wsa);
#endif
    struct addrinfo hints;
    memset(&hints, 0, sizeof hints);
    hints.ai_family = AF_UNSPEC;
    hints.ai_socktype = SOCK_STREAM;
    struct addrinfo *res = nullptr;
    if (getaddrinfo(host, port, &hints, &res) != 0 || !res)
        return BAD_SOCK;
    sock_t s = BAD_SOCK;
    for (struct addrinfo *ai = res; ai; ai = ai->ai_next) {
        s = socket(ai->ai_family, ai->ai_socktype, ai->ai_protocol);
        if (s == BAD_SOCK)
            continue;
        if (connect(s, ai->ai_addr, (int)ai->ai_addrlen) == 0)
            break;
#if defined(_WIN32)
        closesocket(s);
#else
        close(s);
#endif
        s = BAD_SOCK;
    }
    freeaddrinfo(res);
    return s;
}

int main(int argc, char *argv[]) {
    if (argc < 3) {
        fprintf(stderr, "Usage: %s <host> <port>\n", argv[0]);
        return 2;
    }
    const char *host = argv[1];
    const char *port = argv[2];

    sock_t sock = connectTo(host, port);
    if (sock == BAD_SOCK) {
        fprintf(stderr, "i8085-console: cannot connect to %s:%s\n", host, port);
        fprintf(stderr, "(press Enter to close)\n");
        getchar();
        return 1;
    }
    fprintf(stderr, "--- i8085-console connected to %s:%s (close window to detach) ---\r\n", host, port);

#if defined(_WIN32)
    HANDLE hIn = GetStdHandle(STD_INPUT_HANDLE);
    DWORD savedMode = 0;
    GetConsoleMode(hIn, &savedMode);
    // Raw: no line buffering, no echo. Keep processed input so Ctrl-C works.
    SetConsoleMode(hIn, (savedMode & ~(ENABLE_LINE_INPUT | ENABLE_ECHO_INPUT)) | ENABLE_PROCESSED_INPUT);
    _setmode(_fileno(stdout), _O_BINARY);
#else
    struct termios saved;
    tcgetattr(STDIN_FILENO, &saved);
    struct termios raw = saved;
    raw.c_lflag &= ~(ICANON | ECHO); // keep ISIG so Ctrl-C exits
    raw.c_cc[VMIN] = 1;
    raw.c_cc[VTIME] = 0;
    tcsetattr(STDIN_FILENO, TCSANOW, &raw);
#endif

    // Reader thread: socket -> stdout.
    std::thread reader([sock]() {
        char buf[512];
        for (;;) {
            int n = (int)recv(sock, buf, sizeof buf, 0);
            if (n <= 0)
                break;
            fwrite(buf, 1, (size_t)n, stdout);
            fflush(stdout);
        }
    });

    // Main loop: stdin -> socket, one byte at a time (raw).
    for (;;) {
        char ch;
#if defined(_WIN32)
        DWORD n = 0;
        if (!ReadFile(hIn, &ch, 1, &n, nullptr) || n == 0)
            break;
#else
        ssize_t n = read(STDIN_FILENO, &ch, 1);
        if (n <= 0)
            break;
#endif
        if (send(sock, &ch, 1, 0) <= 0)
            break;
    }

#if defined(_WIN32)
    shutdown(sock, SD_BOTH);
    closesocket(sock);
    SetConsoleMode(hIn, savedMode);
#else
    shutdown(sock, SHUT_RDWR);
    close(sock);
    tcsetattr(STDIN_FILENO, TCSANOW, &saved);
#endif
    reader.join();
    return 0;
}
