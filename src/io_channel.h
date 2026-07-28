//----------------------------------------------------------------------------
// io_channel.h - core byte-channel services offered to I/O plugins.
//
// The core owns all the platform plumbing (TCP sockets, terminal-window
// spawning, stdout framing); plugins reach it through the I8085HostAPI byte-
// channel abstraction (see i8085_io_plugin.h). This keeps UART-style plugins
// free of any socket / #ifdef _WIN32 code.
//----------------------------------------------------------------------------

#pragma once

#include "i8085_io_plugin.h"

#ifdef __cplusplus
extern "C" {
#endif

// Host-services table to hand to a plugin's init2.
const I8085HostAPI *io_channels_host_api(void);

// Record the simulator executable path (argv[0]) so "window" channels can
// locate the bundled i8085-console helper next to it.
void io_channels_set_exe_path(const char *argv0);

// Service all open channels once per CPU step: accept pending clients, drain
// received bytes, and idle-flush line-buffered mirrors.
void io_channels_tick(UINT64 step);

// Close every open channel (shutdown).
void io_channels_shutdown(void);

#ifdef __cplusplus
}
#endif
