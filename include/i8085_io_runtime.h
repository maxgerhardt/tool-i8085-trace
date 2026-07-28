#pragma once

#include "i8085_cpu.h"
#include <stddef.h>

#ifdef __cplusplus
extern "C" {
#endif

void io_runtime_set_trace(int enabled);
// Append a plugin instance. Repeatable: one call per --io-plugin/-config pair.
int io_runtime_load_plugin(const char *path, const char *config, char *errbuf, size_t errbuf_len);
// Unload ALL loaded plugins.
void io_runtime_unload_plugin(void);
void io_runtime_set_state(State8085 *state);
void io_runtime_on_reset(void);
void io_runtime_on_step(UINT64 step, UINT64 tstates);
// Route a CPU memory write through the loaded plugins; returns the value to
// actually store (a memory plugin may veto by returning the existing byte).
UINT8 io_runtime_mem_write(State8085 *state, int addr, int val);

#ifdef __cplusplus
}
#endif
