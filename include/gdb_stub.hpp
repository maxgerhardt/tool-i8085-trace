//----------------------------------------------------------------------------
// I8085 Trace - GDB Remote Serial Protocol stub
//
// gdb_stub.hpp - Header for RSP server
//----------------------------------------------------------------------------

#pragma once

#include "types.h"
#include <vector>

//----------------------------------------------------------------------------
// Periodic timer (repeating interrupt at fixed T-state intervals)
// Shared between main.cpp and gdb_stub.cpp
//----------------------------------------------------------------------------

struct PeriodicTimer {
    int code;
    UINT64 periodCycles;
    UINT64 nextTriggerCycle;
};

struct State8085;
struct ExecutionStats8085;

/// Run the GDB RSP server on the given TCP port.
/// Blocks until the client disconnects or sends a kill command.
/// Returns 0 on success, nonzero on error.
int gdb_main(int port, State8085 *state, ExecutionStats8085 *stats, std::vector<PeriodicTimer> &timers);
