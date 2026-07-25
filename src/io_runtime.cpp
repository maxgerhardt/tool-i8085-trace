//----------------------------------------------------------------------------
// I8085 Trace - Runtime I/O plugin loader
//
// io_runtime.cpp - Optional dynamic I/O plugin integration
//----------------------------------------------------------------------------

#include "i8085_io_runtime.h"
#include "i8085_io_plugin.h"
#include "disk_emu.h"

#include <cstdio>
#include <cstring>

//----------------------------------------------------------------------------
// Cross-platform dynamic library loading
//
// POSIX uses dlopen/dlsym/dlclose/dlerror from <dlfcn.h>; Windows uses
// LoadLibrary/GetProcAddress/FreeLibrary from <windows.h>. The wrappers below
// keep the loader logic identical on both platforms.
//----------------------------------------------------------------------------

#ifdef _WIN32
#include <windows.h>

static void *dl_open(const char *path) { return (void *)LoadLibraryA(path); }
static void *dl_sym(void *module, const char *symbol) {
    return (void *)GetProcAddress((HMODULE)module, symbol);
}
static void dl_close(void *module) { FreeLibrary((HMODULE)module); }
static const char *dl_error(void) {
    static char buf[256];
    DWORD err = GetLastError();
    DWORD n = FormatMessageA(FORMAT_MESSAGE_FROM_SYSTEM | FORMAT_MESSAGE_IGNORE_INSERTS, nullptr, err, 0, buf,
                             sizeof(buf), nullptr);
    if (n == 0)
        snprintf(buf, sizeof(buf), "error %lu", (unsigned long)err);
    return buf;
}
#else
#include <dlfcn.h>

static void *dl_open(const char *path) { return dlopen(path, RTLD_NOW | RTLD_LOCAL); }
static void *dl_sym(void *module, const char *symbol) { return dlsym(module, symbol); }
static void dl_close(void *module) { dlclose(module); }
static const char *dl_error(void) { return dlerror(); }
#endif

struct IORuntimeState {
    bool ioTrace = false;
    State8085 *state = nullptr;
    void *module = nullptr;
    void *pluginCtx = nullptr;
    I8085IoPluginAPI api = {};
};

static IORuntimeState gRuntime;

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
    if (gRuntime.state && gRuntime.module && gRuntime.api.on_reset) {
        gRuntime.api.on_reset(gRuntime.pluginCtx, gRuntime.state);
    }
}

void io_runtime_on_step(UINT64 step, UINT64 tstates) {
    if (gRuntime.state && gRuntime.module && gRuntime.api.on_step) {
        gRuntime.api.on_step(gRuntime.pluginCtx, gRuntime.state, step, tstates);
    }
}

int io_runtime_load_plugin(const char *path, const char *config, char *errbuf, size_t errbuf_len) {
    if (!path || *path == '\0') {
        SetErr(errbuf, errbuf_len, "empty plugin path");
        return -1;
    }

    io_runtime_unload_plugin();

    void *module = dl_open(path);
    if (!module) {
        SetErr(errbuf, errbuf_len, dl_error());
        return -1;
    }

    auto initFn = (I8085IoPluginInitFn)dl_sym(module, I8085_IO_PLUGIN_INIT_SYMBOL);
    if (!initFn) {
        SetErr(errbuf, errbuf_len, "plugin init symbol not found");
        dl_close(module);
        return -1;
    }

    void *pluginCtx = nullptr;
    I8085IoPluginAPI api = {};
    char pluginErr[256] = {0};
    int rc = initFn(config, &pluginCtx, &api, pluginErr, sizeof(pluginErr));
    if (rc != 0) {
        if (pluginErr[0] != '\0')
            SetErr(errbuf, errbuf_len, pluginErr);
        else
            SetErr(errbuf, errbuf_len, "plugin init failed");
        dl_close(module);
        return -1;
    }

    if (api.abi_version != I8085_IO_PLUGIN_ABI_VERSION) {
        SetErr(errbuf, errbuf_len, "plugin ABI version mismatch");
        if (api.destroy)
            api.destroy(pluginCtx);
        dl_close(module);
        return -1;
    }

    gRuntime.module = module;
    gRuntime.pluginCtx = pluginCtx;
    gRuntime.api = api;
    return 0;
}

void io_runtime_unload_plugin(void) {
    if (gRuntime.module) {
        if (gRuntime.api.destroy) {
            gRuntime.api.destroy(gRuntime.pluginCtx);
        }
        dl_close(gRuntime.module);
    }
    gRuntime.module = nullptr;
    gRuntime.pluginCtx = nullptr;
    gRuntime.api = {};
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

    if (gRuntime.state && gRuntime.module && gRuntime.api.on_io_write) {
        gRuntime.api.on_io_write(gRuntime.pluginCtx, gRuntime.state, port, val);
    }
}

extern "C" void io_pre_read(int address) {
    UINT8 port = (UINT8)(address & 0xFF);

    if (disk_emu_active() && gRuntime.state) {
        disk_emu_on_io_pre_read(gRuntime.state, port);
    }

    if (gRuntime.state && gRuntime.module && gRuntime.api.on_io_pre_read) {
        gRuntime.api.on_io_pre_read(gRuntime.pluginCtx, gRuntime.state, port);
    }
}

extern "C" void io_read(int address, int value) {
    UINT8 port = (UINT8)(address & 0xFF);
    UINT8 val = (UINT8)(value & 0xFF);

    if (gRuntime.ioTrace) {
        fprintf(stderr, "[IO] IN  0x%02X -> 0x%02X\n", port, val);
    }

    if (gRuntime.state && gRuntime.module && gRuntime.api.on_io_post_read) {
        gRuntime.api.on_io_post_read(gRuntime.pluginCtx, gRuntime.state, port, val);
    }
}
