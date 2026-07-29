#pragma once

#include "i8085_cpu.h"
#include <stddef.h>

#ifdef __cplusplus
extern "C" {
#endif

#define I8085_IO_PLUGIN_ABI_VERSION 2u
// The single entry point every plugin must export (see I8085IoPluginInitFn).
#define I8085_IO_PLUGIN_INIT_SYMBOL "i8085_io_plugin_init"
#define I8085_HOST_API_VERSION 1u

// Host services handed to a plugin at init. The core owns all the platform
// plumbing (sockets, terminal spawning, stdout framing) behind a simple byte-
// channel abstraction, so a plugin that needs a UART backend does not carry any
// #ifdef _WIN32 socket code -- it just opens channels and pumps bytes.
typedef struct I8085HostAPI {
    UINT32 abi_version;
    // Open a byte channel from a spec string:
    //   "tcp=PORT"       listen on localhost:PORT, bridge one client
    //   "window[=PORT]"  like tcp (PORT optional -> auto-assign) and also spawn
    //                    an i8085-console terminal window bound to it
    //   "gdbtx"          write-only, GDB/MI-safe stdout mirror (CR-free, LF-framed)
    //   "stdout"         write-only raw stdout mirror
    // Returns an opaque channel handle, or NULL on error (errbuf filled).
    void *(*channel_open)(const char *spec, char *errbuf, size_t errbuf_len);
    // Nonblocking: copy up to maxlen received bytes into buf; returns the count
    // (0 if none currently available). Write-only channels always return 0.
    int (*channel_read)(void *chan, void *buf, int maxlen);
    // Queue bytes to transmit; returns the number accepted.
    int (*channel_write)(void *chan, const void *buf, int len);
    void (*channel_close)(void *chan);
} I8085HostAPI;

// The API struct is append-only: new callbacks are only ever added at the END,
// and the host zero-initializes the struct before calling a plugin's init and
// null-checks every callback before use. A future ABI revision bumps
// I8085_IO_PLUGIN_ABI_VERSION; the host requires plugins to report exactly the
// current version.
typedef struct I8085IoPluginAPI {
    UINT32 abi_version;
    void (*destroy)(void *ctx);
    void (*on_reset)(void *ctx, State8085 *state);
    void (*on_step)(void *ctx, State8085 *state, UINT64 step, UINT64 tstates);
    void (*on_io_write)(void *ctx, State8085 *state, UINT8 port, UINT8 value);
    void (*on_io_pre_read)(void *ctx, State8085 *state, UINT8 port);
    void (*on_io_post_read)(void *ctx, State8085 *state, UINT8 port, UINT8 value);

    // Called for every CPU memory write, BEFORE the byte is stored.
    // Returns the value to actually store: return `val` to allow the write, or
    // return the current state->memory[addr] to veto it (write-protection).
    // When several plugins are loaded the value is chained through each in load
    // order. Plugins that do not model memory leave this NULL.
    UINT8 (*on_mem_write)(void *ctx, State8085 *state, UINT16 addr, UINT8 val);
} I8085IoPluginAPI;

// Every plugin exports i8085_io_plugin_init with this signature. `host` is the
// host byte-channel table (never NULL); a plugin that needs no host services
// simply ignores it.
typedef int (*I8085IoPluginInitFn)(const char *config, const I8085HostAPI *host, void **out_ctx,
                                   I8085IoPluginAPI *out_api, char *errbuf, size_t errbuf_len);

#ifdef __cplusplus
}
#endif
