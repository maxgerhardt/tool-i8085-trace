#pragma once

#include "i8085_cpu.h"
#include <stddef.h>

#ifdef __cplusplus
extern "C" {
#endif

#define I8085_IO_PLUGIN_ABI_VERSION 1u
#define I8085_IO_PLUGIN_INIT_SYMBOL "i8085_io_plugin_init"

typedef struct I8085IoPluginAPI {
    UINT32 abi_version;
    void (*destroy)(void *ctx);
    void (*on_reset)(void *ctx, State8085 *state);
    void (*on_step)(void *ctx, State8085 *state, UINT64 step, UINT64 tstates);
    void (*on_io_write)(void *ctx, State8085 *state, UINT8 port, UINT8 value);
    void (*on_io_pre_read)(void *ctx, State8085 *state, UINT8 port);
    void (*on_io_post_read)(void *ctx, State8085 *state, UINT8 port, UINT8 value);
} I8085IoPluginAPI;

typedef int (*I8085IoPluginInitFn)(const char *config, void **out_ctx, I8085IoPluginAPI *out_api, char *errbuf,
                                   size_t errbuf_len);

#ifdef __cplusplus
}
#endif
