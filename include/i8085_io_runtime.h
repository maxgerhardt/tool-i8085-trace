#pragma once

#include "i8085_cpu.h"
#include <stddef.h>

#ifdef __cplusplus
extern "C" {
#endif

void io_runtime_set_trace(int enabled);
int io_runtime_load_plugin(const char *path, const char *config, char *errbuf, size_t errbuf_len);
void io_runtime_unload_plugin(void);
void io_runtime_set_state(State8085 *state);
void io_runtime_on_reset(void);
void io_runtime_on_step(UINT64 step, UINT64 tstates);

#ifdef __cplusplus
}
#endif
