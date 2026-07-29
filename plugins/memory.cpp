//----------------------------------------------------------------------------
// memory.cpp - i8085-trace memory-region plugin (ROM / write-protected EEPROM)
//
// Enforces a write-protected memory region through the ABI v2 on_mem_write
// hook: while the region is protected, every CPU write to it is vetoed by
// returning the byte already in memory (so /DATA-polling loaders detect the
// write-protect and firmware cannot corrupt ROM). Stateless -- no shadow
// buffer, no per-step region scan.
//
// Config string (';'-separated key=value):
//   base=<addr>      region base address (0x.. hex or decimal), default 0
//   size=<bytes>     region size, default 0x8000
//   mode=rom|eeprom  rom    = always write-protected;
//                    eeprom = write-protected unless wren is on
//   wren=on|off      eeprom initial write-enable (default off = protected)
//----------------------------------------------------------------------------

#include "i8085_io_plugin.h"

#include <cstdlib>
#include <string>

#if defined(_WIN32)
#define PLUGIN_EXPORT extern "C" __declspec(dllexport)
#else
#define PLUGIN_EXPORT extern "C" __attribute__((visibility("default")))
#endif

struct Ctx {
    UINT32 base = 0;
    UINT32 end = 0x8000; // exclusive
    bool eeprom = false;  // false = ROM (always protected); true = wren-gated
    bool wren = false;    // eeprom write-enable
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
    return (UINT32)strtoul(s.c_str(), nullptr, 0); // base 0: honour 0x / decimal
}

static bool truthy(const std::string &s) {
    return s == "on" || s == "1" || s == "true" || s == "yes";
}

// ABI v2: veto a write to a protected region by returning the existing byte.
static UINT8 on_mem_write(void *vctx, State8085 *state, UINT16 addr, UINT8 val) {
    Ctx *c = (Ctx *)vctx;
    if (addr >= c->base && addr < c->end) {
        bool protectedNow = c->eeprom ? !c->wren : true;
        if (protectedNow)
            return state->memory[addr]; // write-protected: keep the old value
    }
    return val;
}

static void destroy(void *vctx) {
    delete (Ctx *)vctx;
}

static int snapshot(void *vctx, I8085PluginInfo *info, I8085StateField *f, int max) {
    Ctx *c = (Ctx *)vctx;
    if (info) {
        info->kind = "memory";
        info->base = (UINT16)c->base;
        info->span = (UINT16)(c->end - c->base);
    }
    bool protectedNow = c->eeprom ? !c->wren : true;
    int n = 0;
    if (n < max) {
        f[n] = {"mode", I8085_FIELD_STR, 0, c->eeprom ? "eeprom" : "rom"};
        n++;
    }
    if (n < max) {
        f[n] = {"size", I8085_FIELD_HEX, c->end - c->base, nullptr};
        n++;
    }
    if (n < max) {
        f[n] = {"wren", I8085_FIELD_BOOL, c->wren ? 1u : 0u, nullptr};
        n++;
    }
    if (n < max) {
        f[n] = {"prot", I8085_FIELD_BOOL, protectedNow ? 1u : 0u, nullptr};
        n++;
    }
    return n;
}

PLUGIN_EXPORT int i8085_io_plugin_init(const char *config, const I8085HostAPI *host, void **out_ctx,
                                       I8085IoPluginAPI *out_api, char *errbuf, size_t errbuf_len) {
    (void)host; // memory model needs no host services
    Ctx *c = new Ctx();
    UINT32 base = parseNum(cfgValue(config, "base"), 0);
    UINT32 size = parseNum(cfgValue(config, "size"), 0x8000);
    c->base = base;
    c->end = base + size;
    c->eeprom = (cfgValue(config, "mode") == "eeprom");
    c->wren = truthy(cfgValue(config, "wren"));

    out_api->abi_version = I8085_IO_PLUGIN_ABI_VERSION;
    out_api->destroy = destroy;
    out_api->on_mem_write = on_mem_write;
    out_api->snapshot = snapshot;

    *out_ctx = c;
    (void)errbuf;
    (void)errbuf_len;
    return 0;
}
