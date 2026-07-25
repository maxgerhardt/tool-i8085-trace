#pragma once

#include "types.h"
#include <stddef.h>
#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

// Flags (8085)
typedef struct Flags {
    UINT8 z : 1;
    UINT8 s : 1;
    UINT8 p : 1;
    UINT8 cy : 1;
    UINT8 ac : 1;
    UINT8 v : 1;     // Overflow flag (undocumented, PSW bit 1)
    UINT8 x5 : 1;    // X5 flag (undocumented, PSW bit 5)
    UINT8 pad : 1;
} Flags;

// CPU state
typedef struct State8085 {
    UINT8 a;
    UINT8 b;
    UINT8 c;
    UINT8 d;
    UINT8 e;
    UINT8 h;
    UINT8 l;
    UINT16 sp;
    UINT16 pc;
    Flags cc;
    UINT8 int_enable;
    UINT8 int_enable_delay;
    UINT8 r5_mask, r6_mask, r7_mask;
    UINT8 pending_trap, pending_r5, pending_r6, r7_latch;
    UINT8 pending_int;      // 8080-style INT pending (RST 0-7)
    UINT8 int_rst_number;   // Which RST to execute (0-7) when pending_int fires
    UINT8 sid_line;
    UINT8 sod_line;
    UINT8 hlt_enable;
    UINT8 *memory;
    UINT8 *io;
} State8085;

// Execution stats (t-states, stack watermark)
typedef struct ExecutionStats8085 {
    UINT64 total_tstates;
    UINT16 min_sp;
    bool min_sp_set;
} ExecutionStats8085;

// Init/free
State8085 *Init8085(void);
void Free8085(State8085 *state);
void Reset8085(State8085 *state, UINT16 pc, UINT16 sp);

// Step one instruction. Returns 1 on HLT, 0 otherwise.
int Emulate8085Op(State8085 *state, ExecutionStats8085 *stats);

// Disassemble instruction at pc into out buffer. Returns instruction length.
int Disassemble8085Op(const UINT8 *codebuffer, int pc, char *out, size_t out_len);

// Accessors
UINT8 *getMemory(State8085 *state);
UINT8 *getIO(State8085 *state);

// Interrupt trigger
int triggerInterrupt(State8085 *state, int code, int active);

// Serial input/output lines
void setSIDLine(State8085 *state, int level);
int getSODLine(State8085 *state);

// Optional I/O hooks for I/O instructions
void io_write(int address, int value);
void io_pre_read(int address);
void io_read(int address, int value);

#ifdef __cplusplus
}
#endif
