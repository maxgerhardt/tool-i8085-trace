//----------------------------------------------------------------------------
// i8255.cpp - i8085-trace I/O plugin: Intel 8255 PPI (parallel I/O)
//
// A Programmable Peripheral Interface with three 8-bit ports and a control
// register, on a relocatable 4-port window:
//   base+0  Port A (R/W)
//   base+1  Port B (R/W)
//   base+2  Port C (R/W; upper/lower nibbles independently directed)
//   base+3  control word (W): mode-set (bit7=1) or Port-C bit set/reset (bit7=0)
//
// Modeled: Mode 0 (basic I/O). A mode-set control word selects the direction of
// Port A, Port B and the two Port C nibbles, and (per the datasheet) clears the
// output latches. A BSR control word sets/resets an individual Port C output
// bit. Output ports read back their latch; input ports read external pin state,
// which -- with no real hardware wired -- is taken from config (pa/pb/pc).
// Group modes 1/2 (strobed / bidirectional handshaking) are not modeled.
//
// Config string (';'-separated key=value):
//   base=<port>   base I/O port (0x.. or decimal), default 0x00
//   pa=<byte>     external pin value seen when Port A is an input (default 0)
//   pb=<byte>     external pin value seen when Port B is an input (default 0)
//   pc=<byte>     external pin value seen on input Port C nibbles (default 0)
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
    UINT8 base = 0x00;
    UINT8 ctrl = 0x9B; // power-on default: all ports input, mode 0

    // Directions (true = input). Reset default: all input.
    bool aIn = true, bIn = true, cUpperIn = true, cLowerIn = true;
    UINT8 groupAmode = 0, groupBmode = 0;

    UINT8 outA = 0, outB = 0, outC = 0; // output latches
    UINT8 inA = 0, inB = 0, inC = 0;    // external input pin values (from config)
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

static UINT32 parseNum(const std::string &s, UINT32 def) {
    if (s.empty())
        return def;
    return (UINT32)strtoul(s.c_str(), nullptr, 0);
}

// Effective value read from a port: output latch for output ports, external pin
// state for input ports. Port C combines its two independently-directed nibbles.
static UINT8 readPort(const Ctx *c, int i) {
    switch (i) {
    case 0:
        return c->aIn ? c->inA : c->outA;
    case 1:
        return c->bIn ? c->inB : c->outB;
    case 2: {
        UINT8 upper = c->cUpperIn ? (c->inC & 0xF0) : (c->outC & 0xF0);
        UINT8 lower = c->cLowerIn ? (c->inC & 0x0F) : (c->outC & 0x0F);
        return upper | lower;
    }
    default:
        return c->ctrl; // control register is write-only; report last word
    }
}

static void writeControl(Ctx *c, UINT8 val) {
    if (val & 0x80) {
        // Mode-set: define directions/modes and clear the output latches.
        c->ctrl = val;
        c->groupAmode = (val >> 5) & 0x03;
        c->aIn = (val >> 4) & 1;
        c->cUpperIn = (val >> 3) & 1;
        c->groupBmode = (val >> 2) & 1;
        c->bIn = (val >> 1) & 1;
        c->cLowerIn = val & 1;
        c->outA = c->outB = c->outC = 0;
    } else {
        // Bit set/reset on Port C.
        UINT8 bit = (val >> 1) & 0x07;
        if (val & 1)
            c->outC |= (UINT8)(1u << bit);
        else
            c->outC &= (UINT8)~(1u << bit);
    }
}

static void on_io_pre_read(void *vctx, State8085 *state, UINT8 port) {
    Ctx *c = (Ctx *)vctx;
    int i = (int)port - (int)c->base;
    if (i >= 0 && i < 4)
        state->io[port] = readPort(c, i);
}

static void on_io_write(void *vctx, State8085 *state, UINT8 port, UINT8 value) {
    Ctx *c = (Ctx *)vctx;
    (void)state;
    int i = (int)port - (int)c->base;
    switch (i) {
    case 0:
        c->outA = value;
        break;
    case 1:
        c->outB = value;
        break;
    case 2:
        c->outC = value;
        break;
    case 3:
        writeControl(c, value);
        break;
    default:
        break;
    }
}

static void on_reset(void *vctx, State8085 *state) {
    Ctx *c = (Ctx *)vctx;
    (void)state;
    c->ctrl = 0x9B;
    c->aIn = c->bIn = c->cUpperIn = c->cLowerIn = true;
    c->groupAmode = c->groupBmode = 0;
    c->outA = c->outB = c->outC = 0;
}

static int snapshot(void *vctx, I8085PluginInfo *info, I8085StateField *f, int max) {
    Ctx *c = (Ctx *)vctx;
    if (info) {
        info->kind = "i8255";
        info->base = c->base;
        info->span = 4;
    }
    int n = 0;
    auto add = [&](const char *name, UINT8 kind, UINT32 u, const char *s) {
        if (n < max) {
            f[n] = {name, kind, u, s};
            n++;
        }
    };
    add("PA", I8085_FIELD_HEX, readPort(c, 0), nullptr);
    add("PB", I8085_FIELD_HEX, readPort(c, 1), nullptr);
    add("PC", I8085_FIELD_HEX, readPort(c, 2), nullptr);
    add("Adir", I8085_FIELD_STR, 0, c->aIn ? "in" : "out");
    add("Bdir", I8085_FIELD_STR, 0, c->bIn ? "in" : "out");
    add("ctrl", I8085_FIELD_HEX, c->ctrl, nullptr);
    return n;
}

static void destroy(void *vctx) {
    delete (Ctx *)vctx;
}

PLUGIN_EXPORT int i8085_io_plugin_init(const char *config, const I8085HostAPI *host, void **out_ctx,
                                       I8085IoPluginAPI *out_api, char *errbuf, size_t errbuf_len) {
    (void)host; // the PPI needs no host services
    (void)errbuf;
    (void)errbuf_len;
    Ctx *c = new Ctx();
    c->base = (UINT8)(parseNum(cfgValue(config, "base"), 0x00) & 0xFF);
    c->inA = (UINT8)parseNum(cfgValue(config, "pa"), 0);
    c->inB = (UINT8)parseNum(cfgValue(config, "pb"), 0);
    c->inC = (UINT8)parseNum(cfgValue(config, "pc"), 0);

    out_api->abi_version = I8085_IO_PLUGIN_ABI_VERSION;
    out_api->destroy = destroy;
    out_api->on_reset = on_reset;
    out_api->on_io_write = on_io_write;
    out_api->on_io_pre_read = on_io_pre_read;
    out_api->snapshot = snapshot;
    *out_ctx = c;
    return 0;
}
