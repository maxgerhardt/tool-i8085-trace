//----------------------------------------------------------------------------
// boardview.cpp - i8085-trace I/O plugin: generic live board viewer
//
// Periodically enumerates every other loaded plugin via the host peer API and
// renders whatever live state each one self-describes (host/plugin API v3
// snapshot). It hardcodes NO chip knowledge -- a peripheral appears the moment
// it implements snapshot(), so this same viewer shows an MC6850's TX/RX, a
// memory region's protection, or a PIT's free-running counters identically.
//
// Config string (';'-separated key=value):
//   out=<spec>     where to draw. "stdout" (default), "window" (spawns an
//                  i8085-console terminal), "window=<port>", or "tcp=<port>".
//   interval=<n>   redraw every n CPU steps (default 100000)
//   ansi=on|off    prefix each frame with an ANSI clear+home so a window shows
//                  a live-updating panel instead of scrolling (default off)
//----------------------------------------------------------------------------

#include "i8085_io_plugin.h"

#include <cstdio>
#include <cstdlib>
#include <string>

#if defined(_WIN32)
#define PLUGIN_EXPORT extern "C" __declspec(dllexport)
#else
#define PLUGIN_EXPORT extern "C" __attribute__((visibility("default")))
#endif

struct Ctx {
    const I8085HostAPI *host = nullptr;
    void *chan = nullptr;
    UINT64 interval = 100000;
    UINT64 nextRender = 0;
    bool ansi = false;
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

static void appendField(std::string &line, const I8085StateField &f) {
    char buf[128];
    switch (f.kind) {
    case I8085_FIELD_HEX:
        snprintf(buf, sizeof buf, " %s=0x%X", f.name, f.u);
        break;
    case I8085_FIELD_BOOL:
        snprintf(buf, sizeof buf, " %s=%d", f.name, f.u ? 1 : 0);
        break;
    case I8085_FIELD_STR:
        snprintf(buf, sizeof buf, " %s=%s", f.name, f.s ? f.s : "");
        break;
    default: // U8 / U16 / U32
        snprintf(buf, sizeof buf, " %s=%u", f.name, f.u);
        break;
    }
    line += buf;
}

// One pin's glyph. Inputs show an inward arrow + the externally applied level;
// outputs show the driven level, "glowing" bright for 1 / dim for 0 under ANSI.
static void appendPin(std::string &row, bool input, bool hiz, bool level, bool ansi) {
    char buf[24];
    if (hiz) {
        row += ansi ? "\x1b[2m--\x1b[0m" : "z-";
    } else if (input) {
        snprintf(buf, sizeof buf, ansi ? "\x1b[36m>%d\x1b[0m" : "i%d", level ? 1 : 0);
        row += buf;
    } else {
        snprintf(buf, sizeof buf, ansi ? (level ? "\x1b[1;32m%d\x1b[0m" : "\x1b[2m%d\x1b[0m") : "o%d",
                 level ? 1 : 0);
        row += buf;
    }
}

static void appendPinBus(std::string &out, const char *name, const I8085PinBus *b, bool ansi) {
    if (!b)
        return;
    char lead[24];
    snprintf(lead, sizeof lead, "    %-4s ", name ? name : "");
    std::string row = lead;
    for (int bit = (int)b->width - 1; bit >= 0; bit--) { // MSB..LSB, like a byte
        appendPin(row, (b->is_input >> bit) & 1, (b->hi_z >> bit) & 1, (b->level >> bit) & 1, ansi);
        row += ' ';
    }
    out += row;
    out += '\n';
}

static void render(Ctx *c, UINT64 step) {
    if (!c->host || !c->host->peer_count || !c->host->peer_snapshot)
        return;

    std::string frame;
    if (c->ansi)
        frame += "\x1b[2J\x1b[H"; // clear screen + home cursor

    char hdr[64];
    snprintf(hdr, sizeof hdr, "=== i8085 board @ step %llu ===\n", (unsigned long long)step);
    frame += hdr;

    int peers = c->host->peer_count();
    for (int i = 0; i < peers; i++) {
        I8085PluginInfo info = {nullptr, 0, 0};
        I8085StateField fields[32] = {}; // zero-init: PINS `bus` NULL unless set
        int n = c->host->peer_snapshot(i, &info, fields, 32);
        if (n < 0 || !info.kind)
            continue; // out of range, or an opaque peer (incl. this viewer)

        char head[64];
        if (info.span > 0)
            snprintf(head, sizeof head, "[%-8s @0x%02X-0x%02X]", info.kind, info.base,
                     (info.base + info.span - 1) & 0xFFFF);
        else
            snprintf(head, sizeof head, "[%-8s]", info.kind);
        std::string line = head;
        std::string pinRows;
        for (int j = 0; j < n; j++) {
            if (fields[j].kind == I8085_FIELD_PINS)
                appendPinBus(pinRows, fields[j].name, fields[j].bus, c->ansi); // its own row
            else
                appendField(line, fields[j]); // scalar: inline on the header line
        }
        line += "\n";
        frame += line;
        frame += pinRows;
    }

    if (c->host->channel_write)
        c->host->channel_write(c->chan, frame.data(), (int)frame.size());
}

static void on_step(void *vctx, State8085 *state, UINT64 step, UINT64 tstates) {
    Ctx *c = (Ctx *)vctx;
    (void)state;
    (void)tstates;
    if (step >= c->nextRender) {
        render(c, step);
        c->nextRender = step + c->interval;
    }
}

static void destroy(void *vctx) {
    Ctx *c = (Ctx *)vctx;
    if (c && c->host && c->host->channel_close && c->chan)
        c->host->channel_close(c->chan);
    delete c;
}

PLUGIN_EXPORT int i8085_io_plugin_init(const char *config, const I8085HostAPI *host, void **out_ctx,
                                       I8085IoPluginAPI *out_api, char *errbuf, size_t errbuf_len) {
    Ctx *c = new Ctx();
    c->host = host;
    c->interval = strtoull(cfgValue(config, "interval").c_str(), nullptr, 0);
    if (c->interval == 0)
        c->interval = 100000;
    c->ansi = truthy(cfgValue(config, "ansi"));

    std::string out = cfgValue(config, "out");
    if (out.empty())
        out = "stdout";
    if (host && host->channel_open) {
        char err[128] = {0};
        c->chan = host->channel_open(out.c_str(), err, sizeof err);
        if (!c->chan) {
            snprintf(errbuf, errbuf_len, "boardview: channel '%s' failed: %s", out.c_str(), err);
            delete c;
            return -1;
        }
    }

    out_api->abi_version = I8085_IO_PLUGIN_ABI_VERSION;
    out_api->destroy = destroy;
    out_api->on_step = on_step;
    // No snapshot: the viewer reports itself as opaque so it never renders itself.
    *out_ctx = c;
    return 0;
}
