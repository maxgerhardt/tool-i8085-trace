//----------------------------------------------------------------------------
// mc6850.cpp - i8085-trace I/O plugin: MC68B50 (6850) ACIA (UART)
//
// Emulates an MC68B50 ACIA on a relocatable 2-port window: status (R) / control
// (W) at `base`, data (R/W) at `base+1`. Provides a 1-byte receive holding
// register, scripted RX, TX capture, RST 7.5 on receive (when RX interrupt
// enabled) and /RTS flow control. All interactive I/O backends (TCP bridge,
// spawned console window, stdout / GDB-safe mirrors) are provided by the CORE
// via the host byte-channel API, so this plugin carries no socket or platform
// code -- it just opens channels and pumps bytes.
//
// Config string (';'-separated key=value):
//   base=<port>        base I/O port (0x.. or decimal), default 0xDE
//                      (status/control at base, data at base+1)
//   rx=<hexbytes>      bytes to feed the CPU (e.g. "41 42 0D" or "41420D")
//   rxfile=<path>      binary file of RX bytes (alternative to rx=)
//   txlog=<path>       append every transmitted byte to a file (binary)
//   txlog=-            stream transmitted bytes live to stdout ("stdout" also)
//   rtslog=<path>      append "/RTS" transitions here (text; else stderr)
//   bytegap=<n>        min instruction steps between received bytes (default 64)
//   tcp=<port>         bridge the ACIA to a TCP socket (localhost:<port>)
//   window[=<port>]    like tcp AND auto-spawn an i8085-console terminal window
//                      (uses tcp=<port> if also given, else an auto port)
//   gdbtx=1            mirror transmitted bytes to a GDB/MI-safe stdout stream
//----------------------------------------------------------------------------

#include "i8085_io_plugin.h"

#include <cctype>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <deque>
#include <string>
#include <vector>

#if defined(_WIN32)
#define PLUGIN_EXPORT extern "C" __declspec(dllexport)
#else
#define PLUGIN_EXPORT extern "C" __attribute__((visibility("default")))
#endif

struct Ctx {
    UINT8 ctrlPort = 0xDE; // status (R) / control (W)
    UINT8 dataPort = 0xDF; // rx (R) / tx (W)

    std::vector<UINT8> rx;
    size_t rxPos = 0;
    bool rdrFull = false;
    UINT8 rdrByte = 0;
    UINT64 nextArrival = 0;
    UINT64 bytegap = 64;
    UINT64 lastStep = 0;

    FILE *txlog = nullptr; // txlog=<file> capture (txlog=- uses a stdout channel)
    FILE *rtslog = nullptr;

    bool rxIntEnabled = false;
    bool rtsAsserted = true; // ready to receive by default

    int lastTx = -1; // last byte transmitted by the CPU (-1 = none yet)
    int lastRx = -1; // last byte the CPU read out of the RDR (-1 = none yet)

    // Modem lines (level 1 = asserted). /CTS and /DCD are inputs supplied by
    // config (no real modem wired); display-only, they do not gate TX/RX here.
    int ctsLevel = 1; // /CTS: clear to send (default asserted)
    int dcdLevel = 1; // /DCD: data carrier detect (default asserted)
    I8085PinBus busRTS{}, busIRQ{}, busCTS{}, busDCD{};

    // Interactive / mirror backends, provided by the core (tcp, window, stdout,
    // gdbtx). The plugin reads received bytes from and writes TX bytes to these.
    const I8085HostAPI *host = nullptr;
    std::vector<void *> channels;
    std::deque<UINT8> rxq; // bytes received from channels, feeding the RDR
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

static bool truthy(const std::string &s) {
    return s == "1" || s == "on" || s == "true" || s == "yes";
}

static void parseHexBytes(const std::string &in, std::vector<UINT8> &out) {
    std::string h;
    for (char c : in)
        if (isxdigit((unsigned char)c))
            h.push_back(c);
    for (size_t i = 0; i + 1 < h.size(); i += 2)
        out.push_back((UINT8)strtol(h.substr(i, 2).c_str(), nullptr, 16));
}

static void openChannel(Ctx *c, const std::string &spec) {
    if (!c->host || !c->host->channel_open)
        return;
    char err[128] = {0};
    void *ch = c->host->channel_open(spec.c_str(), err, sizeof err);
    if (ch)
        c->channels.push_back(ch);
    else
        fprintf(stderr, "[mc6850 @0x%02X] channel '%s' failed: %s\n", c->ctrlPort, spec.c_str(), err);
}

static void logRts(Ctx *c, bool asserted, UINT64 step) {
    FILE *f = c->rtslog ? c->rtslog : stderr;
    fprintf(f, "RTS=%d step=%llu\n", asserted ? 1 : 0, (unsigned long long)step);
    if (c->rtslog)
        fflush(c->rtslog);
}

static void destroy(void *vctx) {
    Ctx *c = (Ctx *)vctx;
    if (!c)
        return;
    if (c->host && c->host->channel_close)
        for (void *ch : c->channels)
            c->host->channel_close(ch);
    if (c->txlog)
        fclose(c->txlog);
    if (c->rtslog)
        fclose(c->rtslog);
    delete c;
}

static void on_reset(void *vctx, State8085 *state) {
    Ctx *c = (Ctx *)vctx;
    c->rxPos = 0;
    c->rdrFull = false;
    c->nextArrival = 0;
    c->rxIntEnabled = false;
    c->rtsAsserted = true;
    (void)state;
}

static void on_step(void *vctx, State8085 *state, UINT64 step, UINT64 tstates) {
    Ctx *c = (Ctx *)vctx;
    (void)tstates;
    c->lastStep = step;

    // Drain any bytes the core received on our interactive channels; those feed
    // the RDR below just like scripted input.
    if (c->host && c->host->channel_read) {
        unsigned char buf[64];
        for (void *ch : c->channels) {
            int n = c->host->channel_read(ch, buf, sizeof buf);
            for (int i = 0; i < n; i++)
                c->rxq.push_back(buf[i]);
        }
    }

    // Peek the next scripted byte into the receive holding register. rxPos is
    // only advanced when the byte is actually consumed (read of data port), so a
    // 68B50 master reset that clears an unread byte returns it to the queue.
    if (!c->rdrFull && c->rxPos < c->rx.size() && c->rtsAsserted && step >= c->nextArrival) {
        c->rdrByte = c->rx[c->rxPos];
        c->rdrFull = true;
    }

    // Feed a live (channel) byte into the RDR when free. No bytegap pacing: live
    // input arrives as fast as it is typed.
    if (!c->rdrFull && !c->rxq.empty() && c->rtsAsserted) {
        c->rdrByte = c->rxq.front();
        c->rxq.pop_front();
        c->rdrFull = true;
    }

    // The 6850 /IRQ is level-based on (RDRF & RX-int-enable); assert the 8085's
    // RST 7.5 latch while a byte is pending and RX interrupts are enabled, so a
    // byte that arrived before EI still interrupts once interrupts come on.
    // When a netlist is authoritative (wiring_active), the interrupt reaches
    // the CPU via the modeled /IRQ -> pull-up -> inverter -> RST7.5 wiring
    // instead, so skip the direct poke and defer to the net.
    if (c->rdrFull && c->rxIntEnabled && !(c->host && c->host->wiring_active))
        state->r7_latch = 1;
}

static void on_io_pre_read(void *vctx, State8085 *state, UINT8 port) {
    Ctx *c = (Ctx *)vctx;
    if (port == c->ctrlPort)
        state->io[c->ctrlPort] = 0x02 | (c->rdrFull ? 0x01 : 0x00); // TDRE | RDRF
    else if (port == c->dataPort)
        state->io[c->dataPort] = c->rdrByte;
}

static void on_io_post_read(void *vctx, State8085 *state, UINT8 port, UINT8 value) {
    Ctx *c = (Ctx *)vctx;
    (void)state;
    (void)value;
    if (port == c->dataPort && c->rdrFull) {
        c->lastRx = c->rdrByte;                    // the byte the CPU just read
        c->rdrFull = false;
        c->rxPos++;                                // consumed: advance the queue
        c->nextArrival = c->lastStep + c->bytegap; // pace the next received byte
    }
}

static void on_io_write(void *vctx, State8085 *state, UINT8 port, UINT8 value) {
    Ctx *c = (Ctx *)vctx;
    (void)state;
    if (port == c->dataPort) {
        c->lastTx = value;
        if (c->txlog) {
            fputc(value, c->txlog);
            fflush(c->txlog);
        }
        if (c->host && c->host->channel_write) {
            char b = (char)value;
            for (void *ch : c->channels)
                c->host->channel_write(ch, &b, 1);
        }
        return;
    }
    if (port == c->ctrlPort) {
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

static int snapshot(void *vctx, I8085PluginInfo *info, I8085StateField *f, int max) {
    Ctx *c = (Ctx *)vctx;
    if (info) {
        info->kind = "mc6850";
        info->base = c->ctrlPort;
        info->span = 2;
    }
    // Each modem line is a 1-pin bus so it renders as a named pin (arrow/glow).
    // /IRQ (output) is open-drain, active-low, no internal pull: while a
    // received byte is pending with RX-int enabled it drives low; otherwise it
    // releases to hi-Z so an external pull-up (modeled in the netlist) holds
    // it high when idle.
    bool irqAsserted = (c->rdrFull && c->rxIntEnabled);
    c->busRTS = {1, c->rtsAsserted ? 1u : 0u, 0u, 0u};  // output
    c->busIRQ = irqAsserted ? I8085PinBus{1, 0u, 0u, 0u}   // drive low
                            : I8085PinBus{1, 0u, 0u, 1u};  // hi-Z (released)
    c->busCTS = {1, (UINT32)(c->ctsLevel & 1), 1u, 0u}; // input
    c->busDCD = {1, (UINT32)(c->dcdLevel & 1), 1u, 0u}; // input

    int n = 0;
    auto add = [&](const char *name, UINT8 kind, UINT32 u, const I8085PinBus *bus) {
        if (n < max) {
            f[n] = {name, kind, u, nullptr, bus};
            n++;
        }
    };
    add("TX", I8085_FIELD_HEX, (UINT32)(c->lastTx < 0 ? 0 : c->lastTx), nullptr);
    add("RX", I8085_FIELD_HEX, (UINT32)(c->lastRx < 0 ? 0 : c->lastRx), nullptr);
    add("RDRF", I8085_FIELD_BOOL, c->rdrFull ? 1u : 0u, nullptr);
    add("RTS", I8085_FIELD_PINS, 0, &c->busRTS);
    add("IRQ", I8085_FIELD_PINS, 0, &c->busIRQ);
    add("CTS", I8085_FIELD_PINS, 0, &c->busCTS);
    add("DCD", I8085_FIELD_PINS, 0, &c->busDCD);
    return n;
}

// Net-driven modem input lines. Called by the runtime net engine when
// wiring_active, to set the levels the netlist has resolved for CTS/DCD (this
// plugin's only inputs); an unresolved/contended net (X) is treated as 0.
static void pin_set(void *vctx, const char *pin, UINT8 level) {
    Ctx *c = (Ctx *)vctx;
    int lv = (level == 1) ? 1 : 0; // X treated as 0
    if (!strcmp(pin, "CTS"))
        c->ctsLevel = lv;
    else if (!strcmp(pin, "DCD"))
        c->dcdLevel = lv;
}

PLUGIN_EXPORT int i8085_io_plugin_init(const char *config, const I8085HostAPI *host, void **out_ctx,
                                       I8085IoPluginAPI *out_api, char *errbuf, size_t errbuf_len) {
    Ctx *c = new Ctx();
    c->host = host;

    // Relocatable port window: status/control at base, data at base+1.
    std::string baseStr = cfgValue(config, "base");
    if (!baseStr.empty()) {
        UINT32 base = (UINT32)strtoul(baseStr.c_str(), nullptr, 0);
        c->ctrlPort = (UINT8)(base & 0xFF);
        c->dataPort = (UINT8)((base + 1) & 0xFF);
    }

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
        openChannel(c, "stdout"); // live raw mirror to the host stdout
    } else if (!tx.empty()) {
        c->txlog = fopen(tx.c_str(), "wb");
    }

    std::string rl = cfgValue(config, "rtslog");
    if (!rl.empty())
        c->rtslog = fopen(rl.c_str(), "w");

    std::string bg = cfgValue(config, "bytegap");
    if (!bg.empty())
        c->bytegap = strtoull(bg.c_str(), nullptr, 10);

    // External modem input levels (display-only pins). Default asserted.
    std::string cts = cfgValue(config, "cts");
    if (!cts.empty())
        c->ctsLevel = truthy(cts) ? 1 : (int)strtol(cts.c_str(), nullptr, 0);
    std::string dcd = cfgValue(config, "dcd");
    if (!dcd.empty())
        c->dcdLevel = truthy(dcd) ? 1 : (int)strtol(dcd.c_str(), nullptr, 0);

    if (truthy(cfgValue(config, "gdbtx")))
        openChannel(c, "gdbtx");

    // A windowed UART is a TCP bridge plus a spawned terminal; window subsumes
    // tcp (using the tcp port if one was also given). Plain tcp= just bridges.
    std::string tcp = cfgValue(config, "tcp");
    if (truthy(cfgValue(config, "window"))) {
        openChannel(c, tcp.empty() ? std::string("window") : ("window=" + tcp));
    } else if (!tcp.empty()) {
        openChannel(c, "tcp=" + tcp);
    }

    out_api->abi_version = I8085_IO_PLUGIN_ABI_VERSION;
    out_api->destroy = destroy;
    out_api->on_reset = on_reset;
    out_api->on_step = on_step;
    out_api->on_io_write = on_io_write;
    out_api->on_io_pre_read = on_io_pre_read;
    out_api->on_io_post_read = on_io_post_read;
    out_api->snapshot = snapshot;
    out_api->pin_set = pin_set;
    *out_ctx = c;
    (void)errbuf_len;
    return 0;
}
