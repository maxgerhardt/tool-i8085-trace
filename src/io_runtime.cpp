//----------------------------------------------------------------------------
// I8085 Trace - Runtime I/O plugin loader
//
// io_runtime.cpp - Optional dynamic I/O plugin integration.
//
// Supports loading MULTIPLE plugin instances (e.g. several MC6850 UARTs at
// different base ports plus a memory-model plugin). Each --io-plugin /
// --io-plugin-config pair appends a plugin; every callback fans out to all
// loaded plugins in load order. A plugin must only touch the ports / memory
// regions it owns.
//----------------------------------------------------------------------------

#include "i8085_io_runtime.h"
#include "i8085_io_plugin.h"
#include "io_channel.h"
#include "disk_emu.h"

#include <cstdio>
#include <cstring>
#include <vector>

// Portable dynamic-loader shim: POSIX dlopen on unix, a thin LoadLibrary
// wrapper on Windows (MinGW ships no dlfcn.h).
#if defined(_WIN32)
#include <windows.h>
#define RTLD_NOW 0
#define RTLD_LOCAL 0
static void *dlopen(const char *path, int) { return (void *)LoadLibraryA(path); }
static void *dlsym(void *h, const char *sym) { return (void *)GetProcAddress((HMODULE)h, sym); }
static int dlclose(void *h) { return FreeLibrary((HMODULE)h) ? 0 : -1; }
static const char *dlerror(void) {
    static char buf[160];
    snprintf(buf, sizeof buf, "LoadLibrary/GetProcAddress failed (error %lu)", (unsigned long)GetLastError());
    return buf;
}
#else
#include <dlfcn.h>
#endif

struct PluginSlot {
    void *module = nullptr;
    void *ctx = nullptr;
    I8085IoPluginAPI api = {};
};

struct IORuntimeState {
    bool ioTrace = false;
    State8085 *state = nullptr;
    std::vector<PluginSlot> plugins;
    bool anyMemWrite = false; // true if any loaded plugin implements on_mem_write
};

static IORuntimeState gRuntime;

// --- Peer introspection: back the host API's peer_count / peer_snapshot -------
static int host_peer_count(void) {
    return (int)gRuntime.plugins.size();
}

static int host_peer_snapshot(int idx, I8085PluginInfo *out_info, I8085StateField *out_fields, int max_fields) {
    if (idx < 0 || idx >= (int)gRuntime.plugins.size())
        return -1;
    if (out_info) {
        out_info->kind = nullptr;
        out_info->base = 0;
        out_info->span = 0;
    }
    PluginSlot &p = gRuntime.plugins[(size_t)idx];
    if (!p.api.snapshot)
        return 0; // opaque peer: exists but does not self-describe
    return p.api.snapshot(p.ctx, out_info, out_fields, max_fields);
}

// The host API handed to every plugin: the core byte-channel services (owned by
// io_channel) plus the peer-introspection calls (owned here). Assembled once.
static const I8085HostAPI *runtime_host_api(void) {
    static I8085HostAPI api;
    static bool built = false;
    if (!built) {
        const I8085HostAPI *ch = io_channels_host_api();
        if (ch)
            api = *ch; // channel_open/read/write/close
        api.abi_version = I8085_HOST_API_VERSION;
        api.peer_count = host_peer_count;
        api.peer_snapshot = host_peer_snapshot;
        built = true;
    }
    return &api;
}

static void SetErr(char *errbuf, size_t errbuf_len, const char *msg) {
    if (!errbuf || errbuf_len == 0)
        return;
    if (!msg)
        msg = "unknown error";
    snprintf(errbuf, errbuf_len, "%s", msg);
    errbuf[errbuf_len - 1] = '\0';
}

void io_runtime_set_trace(int enabled) {
    gRuntime.ioTrace = (enabled != 0);
}

void io_runtime_set_state(State8085 *state) {
    gRuntime.state = state;
}

void io_runtime_on_reset(void) {
    if (!gRuntime.state)
        return;
    for (auto &p : gRuntime.plugins) {
        if (p.api.on_reset)
            p.api.on_reset(p.ctx, gRuntime.state);
    }
}

void io_runtime_on_step(UINT64 step, UINT64 tstates) {
    // Service host channels first (accept clients, drain input, idle-flush) so a
    // plugin's on_step sees freshly received bytes.
    io_channels_tick(step);
    if (!gRuntime.state)
        return;
    for (auto &p : gRuntime.plugins) {
        if (p.api.on_step)
            p.api.on_step(p.ctx, gRuntime.state, step, tstates);
    }
}

// Load a plugin and APPEND it to the runtime. Repeatable: each call adds one
// instance. Returns 0 on success.
int io_runtime_load_plugin(const char *path, const char *config, char *errbuf, size_t errbuf_len) {
    if (!path || *path == '\0') {
        SetErr(errbuf, errbuf_len, "empty plugin path");
        return -1;
    }

    void *module = dlopen(path, RTLD_NOW | RTLD_LOCAL);
    if (!module) {
        SetErr(errbuf, errbuf_len, dlerror());
        return -1;
    }

    void *pluginCtx = nullptr;
    I8085IoPluginAPI api = {}; // zero-init: newer callbacks left NULL stay unused
    char pluginErr[256] = {0};

    auto initFn = (I8085IoPluginInitFn)dlsym(module, I8085_IO_PLUGIN_INIT_SYMBOL);
    if (!initFn) {
        SetErr(errbuf, errbuf_len, "plugin init symbol not found");
        dlclose(module);
        return -1;
    }
    int rc = initFn(config, runtime_host_api(), &pluginCtx, &api, pluginErr, sizeof(pluginErr));
    if (rc != 0) {
        if (pluginErr[0] != '\0')
            SetErr(errbuf, errbuf_len, pluginErr);
        else
            SetErr(errbuf, errbuf_len, "plugin init failed");
        dlclose(module);
        return -1;
    }

    if (api.abi_version != I8085_IO_PLUGIN_ABI_VERSION) {
        SetErr(errbuf, errbuf_len, "plugin ABI version mismatch");
        if (api.destroy)
            api.destroy(pluginCtx);
        dlclose(module);
        return -1;
    }

    PluginSlot slot;
    slot.module = module;
    slot.ctx = pluginCtx;
    slot.api = api;
    gRuntime.plugins.push_back(slot);
    if (api.on_mem_write)
        gRuntime.anyMemWrite = true;
    return 0;
}

// Unload ALL loaded plugins (reverse load order).
void io_runtime_unload_plugin(void) {
    for (auto it = gRuntime.plugins.rbegin(); it != gRuntime.plugins.rend(); ++it) {
        if (it->api.destroy)
            it->api.destroy(it->ctx);
        if (it->module)
            dlclose(it->module);
    }
    gRuntime.plugins.clear();
    gRuntime.anyMemWrite = false;
    // Safety net: close any channels a plugin did not close itself.
    io_channels_shutdown();
}

extern "C" void io_write(int address, int value) {
    UINT8 port = (UINT8)(address & 0xFF);
    UINT8 val = (UINT8)(value & 0xFF);

    if (gRuntime.ioTrace) {
        fprintf(stderr, "[IO] OUT 0x%02X = 0x%02X\n", port, val);
    }

    if (disk_emu_active() && gRuntime.state) {
        disk_emu_on_io_write(gRuntime.state, port, val);
    }

    if (gRuntime.state) {
        for (auto &p : gRuntime.plugins) {
            if (p.api.on_io_write)
                p.api.on_io_write(p.ctx, gRuntime.state, port, val);
        }
    }
}

extern "C" void io_pre_read(int address) {
    UINT8 port = (UINT8)(address & 0xFF);

    if (disk_emu_active() && gRuntime.state) {
        disk_emu_on_io_pre_read(gRuntime.state, port);
    }

    if (gRuntime.state) {
        for (auto &p : gRuntime.plugins) {
            if (p.api.on_io_pre_read)
                p.api.on_io_pre_read(p.ctx, gRuntime.state, port);
        }
    }
}

extern "C" void io_read(int address, int value) {
    UINT8 port = (UINT8)(address & 0xFF);
    UINT8 val = (UINT8)(value & 0xFF);

    if (gRuntime.ioTrace) {
        fprintf(stderr, "[IO] IN  0x%02X -> 0x%02X\n", port, val);
    }

    if (gRuntime.state) {
        for (auto &p : gRuntime.plugins) {
            if (p.api.on_io_post_read)
                p.api.on_io_post_read(p.ctx, gRuntime.state, port, val);
        }
    }
}

// Called by the CPU core for every memory write (see mem_wr in i8085_exec.c).
// Chains the value through each memory-modelling plugin so a ROM / write-
// protected EEPROM region can veto the write by returning the existing byte.
// Returns `val` unchanged when no plugin models memory (the hot path).
extern "C" UINT8 io_runtime_mem_write(State8085 *state, int addr, int val) {
    UINT8 v = (UINT8)(val & 0xFF);
    if (!gRuntime.anyMemWrite || !state)
        return v;
    UINT16 a = (UINT16)(addr & 0xFFFF);
    for (auto &p : gRuntime.plugins) {
        if (p.api.on_mem_write)
            v = p.api.on_mem_write(p.ctx, state, a, v);
    }
    return v;
}
