//----------------------------------------------------------------------------
// pit8254.cpp - i8085-trace I/O plugin: Intel 8254 (8253 superset) PIT
//
// A Programmable Interval Timer with three independent 16-bit down-counters and
// a control-word register, on a relocatable 4-port window:
//   base+0  counter 0 (R/W count)
//   base+1  counter 1 (R/W count)
//   base+2  counter 2 (R/W count)
//   base+3  control word (W): SC1 SC0 RW1 RW0 M2 M1 M0 BCD
//
// The counters free-run off the CPU clock (t-states), so their CNT values move
// on their own between CPU accesses -- the motivating case for the board-view
// introspection hook: `snapshot` exposes each live counter, mode and OUT pin.
//
// Modeled: control-word decode, LSB/MSB write sequencing, counter-latch reads,
// LSB-then-MSB read sequencing, and counting for modes 0 / 2 / 3 with their OUT
// behaviour. Modes 1/4/5 count down (OUT simplified). BCD counts as binary.
// The gate input is assumed always enabled (no gate line wired on this board).
//
// Config string (';'-separated key=value):
//   base=<port>   base I/O port (0x.. or decimal), default 0x40
//                 (counters at base..base+2, control word at base+3)
//   clkdiv=<n>    CPU t-states per PIT clock tick (default 1: fastest, so the
//                 counters visibly move). e.g. clkdiv=2 halves the timer rate.
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

struct Counter {
    UINT32 initial = 0;   // programmed initial count (0 => 65536)
    UINT32 count = 0;     // current count value (down-counter)
    UINT8 mode = 0;       // 0..5
    UINT8 rw = 3;         // 0 latch, 1 LSB, 2 MSB, 3 LSB-then-MSB
    bool bcd = false;
    bool out = false;     // OUT pin state
    bool running = false; // an initial count has been loaded

    bool writeMsbNext = false; // rw==3: LSB written, MSB expected next
    UINT8 writeLsb = 0;

    bool latched = false;     // a counter-latch snapshot is pending readout
    UINT16 latchVal = 0;
    bool readMsbNext = false; // rw==3: LSB read, MSB expected next
};

struct Ctx {
    UINT8 base = 0x40;
    UINT32 clkdiv = 1;
    UINT64 lastTstates = 0;
    UINT64 clkAccum = 0; // leftover t-states not yet turned into a PIT tick
    Counter c[3];
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

static UINT32 fullCount(const Counter &k) {
    return k.initial == 0 ? 0x10000u : k.initial;
}

// Advance one counter by a single PIT clock tick.
static void tickCounter(Counter &k) {
    if (!k.running)
        return;
    switch (k.mode) {
    case 2: // rate generator: cycles initial..2, reloads at 1, OUT low for it
        if (k.count <= 1) {
            k.count = fullCount(k);
            k.out = true;
        } else {
            k.count--;
            k.out = (k.count != 1);
        }
        break;
    case 3: // square wave: decrement by two, toggle OUT and reload at terminal
        if (k.count <= 2) {
            k.count = fullCount(k);
            k.out = !k.out;
        } else {
            k.count -= 2;
        }
        break;
    default: // modes 0,1,4,5: count down, wrap; OUT high once terminal reached
        if (k.count == 0)
            k.count = 0xFFFF;
        else
            k.count--;
        if (k.count == 0)
            k.out = true;
        break;
    }
}

static void loadCounter(Counter &k, UINT32 initial) {
    k.initial = initial & 0xFFFF;
    k.count = fullCount(k);
    k.running = true;
    // OUT initial level by mode: mode 0 starts low until terminal count.
    k.out = (k.mode != 0);
}

static void writeControl(Ctx *ctx, UINT8 val) {
    UINT8 sc = (val >> 6) & 0x03;
    if (sc == 0x03)
        return; // read-back command (8254): not modeled in this experiment
    Counter &k = ctx->c[sc];
    UINT8 rw = (val >> 4) & 0x03;
    if (rw == 0x00) {
        // Counter-latch command: freeze the current count for readout.
        if (!k.latched) {
            k.latchVal = (UINT16)k.count;
            k.latched = true;
            k.readMsbNext = false;
        }
        return;
    }
    // Programming a new control word: set mode / access, await an initial count.
    k.rw = rw;
    k.mode = (val >> 1) & 0x07;
    if (k.mode > 5)
        k.mode -= 4; // modes 6,7 alias 2,3
    k.bcd = (val & 0x01) != 0;
    k.running = false;
    k.writeMsbNext = false;
    k.readMsbNext = false;
    k.latched = false;
    k.out = (k.mode != 0); // mode 0: OUT low after control word
}

static void writeCounter(Counter &k, UINT8 val) {
    switch (k.rw) {
    case 1: // LSB only
        loadCounter(k, val);
        break;
    case 2: // MSB only
        loadCounter(k, (UINT32)val << 8);
        break;
    case 3: // LSB then MSB
        if (!k.writeMsbNext) {
            k.writeLsb = val;
            k.writeMsbNext = true;
        } else {
            loadCounter(k, ((UINT32)val << 8) | k.writeLsb);
            k.writeMsbNext = false;
        }
        break;
    default:
        break;
    }
}

// The byte the CPU will read from a counter port (latched snapshot or live
// count), honouring LSB/MSB sequencing. Pure -- sequencing advances in post_read.
static UINT8 readCounterByte(const Counter &k) {
    UINT16 v = k.latched ? k.latchVal : (UINT16)k.count;
    switch (k.rw) {
    case 1:
        return (UINT8)(v & 0xFF);
    case 2:
        return (UINT8)(v >> 8);
    case 3:
        return k.readMsbNext ? (UINT8)(v >> 8) : (UINT8)(v & 0xFF);
    default:
        return (UINT8)(v & 0xFF);
    }
}

static void advanceRead(Counter &k) {
    if (k.rw == 3) {
        if (!k.readMsbNext) {
            k.readMsbNext = true; // LSB just read; MSB next
            return;
        }
        k.readMsbNext = false; // MSB read: sequence complete
    }
    k.latched = false; // single-byte read, or the MSB of a pair, ends the latch
}

static void on_step(void *vctx, State8085 *state, UINT64 step, UINT64 tstates) {
    Ctx *ctx = (Ctx *)vctx;
    (void)state;
    (void)step;
    UINT64 delta = tstates - ctx->lastTstates;
    ctx->lastTstates = tstates;
    ctx->clkAccum += delta;
    UINT64 ticks = ctx->clkAccum / ctx->clkdiv;
    ctx->clkAccum -= ticks * ctx->clkdiv;
    // Bound the per-step work; counters are 16-bit so periodic modes only need
    // at most a couple of wraps to look right on a huge jump.
    if (ticks > 0x40000)
        ticks = 0x40000;
    for (UINT64 t = 0; t < ticks; t++)
        for (int i = 0; i < 3; i++)
            tickCounter(ctx->c[i]);
}

static void on_io_pre_read(void *vctx, State8085 *state, UINT8 port) {
    Ctx *ctx = (Ctx *)vctx;
    int i = (int)port - (int)ctx->base;
    if (i >= 0 && i < 3)
        state->io[port] = readCounterByte(ctx->c[i]);
}

static void on_io_post_read(void *vctx, State8085 *state, UINT8 port, UINT8 value) {
    Ctx *ctx = (Ctx *)vctx;
    (void)state;
    (void)value;
    int i = (int)port - (int)ctx->base;
    if (i >= 0 && i < 3)
        advanceRead(ctx->c[i]);
}

static void on_io_write(void *vctx, State8085 *state, UINT8 port, UINT8 value) {
    Ctx *ctx = (Ctx *)vctx;
    (void)state;
    int i = (int)port - (int)ctx->base;
    if (i >= 0 && i < 3)
        writeCounter(ctx->c[i], value);
    else if (i == 3)
        writeControl(ctx, value);
}

static void on_reset(void *vctx, State8085 *state) {
    Ctx *ctx = (Ctx *)vctx;
    (void)state;
    ctx->lastTstates = 0;
    ctx->clkAccum = 0;
    for (int i = 0; i < 3; i++)
        ctx->c[i] = Counter();
}

static const char *const kCntNames[3] = {"C0", "C1", "C2"};
static const char *const kModeNames[3] = {"mode0", "mode1", "mode2"};
static const char *const kOutNames[3] = {"OUT0", "OUT1", "OUT2"};

static int snapshot(void *vctx, I8085PluginInfo *info, I8085StateField *f, int max) {
    Ctx *ctx = (Ctx *)vctx;
    if (info) {
        info->kind = "pit8254";
        info->base = ctx->base;
        info->span = 4;
    }
    int n = 0;
    for (int i = 0; i < 3 && n + 3 <= max; i++) {
        f[n].name = kCntNames[i];
        f[n].kind = I8085_FIELD_U16;
        f[n].u = ctx->c[i].count;
        f[n].s = nullptr;
        n++;
        f[n].name = kModeNames[i];
        f[n].kind = I8085_FIELD_U8;
        f[n].u = ctx->c[i].mode;
        f[n].s = nullptr;
        n++;
        f[n].name = kOutNames[i];
        f[n].kind = I8085_FIELD_BOOL;
        f[n].u = ctx->c[i].out ? 1 : 0;
        f[n].s = nullptr;
        n++;
    }
    return n;
}

static void destroy(void *vctx) {
    delete (Ctx *)vctx;
}

PLUGIN_EXPORT int i8085_io_plugin_init(const char *config, const I8085HostAPI *host, void **out_ctx,
                                       I8085IoPluginAPI *out_api, char *errbuf, size_t errbuf_len) {
    (void)host; // the PIT needs no host services
    (void)errbuf;
    (void)errbuf_len;
    Ctx *ctx = new Ctx();
    ctx->base = (UINT8)(parseNum(cfgValue(config, "base"), 0x40) & 0xFF);
    ctx->clkdiv = parseNum(cfgValue(config, "clkdiv"), 1);
    if (ctx->clkdiv == 0)
        ctx->clkdiv = 1;

    out_api->abi_version = I8085_IO_PLUGIN_ABI_VERSION;
    out_api->destroy = destroy;
    out_api->on_reset = on_reset;
    out_api->on_step = on_step;
    out_api->on_io_write = on_io_write;
    out_api->on_io_pre_read = on_io_pre_read;
    out_api->on_io_post_read = on_io_post_read;
    out_api->snapshot = snapshot;
    *out_ctx = ctx;
    return 0;
}
