//----------------------------------------------------------------------------
// I8085 Trace - State management
//----------------------------------------------------------------------------

#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "i8085_cpu.h"
State8085 *Init8085(void) {
    State8085 *state = calloc(1, sizeof(State8085));
    if (!state) {
        return NULL;
    }
    state->memory = calloc(1, 0x10000); // 64K
    state->io = calloc(1, 0x100);
    if (!state->memory || !state->io) {
        free(state->memory);
        free(state->io);
        free(state);
        return NULL;
    }
    return state;
}

void Free8085(State8085 *state) {
    if (!state) {
        return;
    }
    free(state->memory);
    free(state->io);
    free(state);
}

void Reset8085(State8085 *state, UINT16 pc, UINT16 sp) {
    if (!state) {
        return;
    }
    UINT8 *mem = state->memory;
    UINT8 *io = state->io;
    memset(state, 0, sizeof(*state));
    state->memory = mem;
    state->io = io;
    state->pc = pc;
    state->sp = sp;
}

UINT8 *getMemory(State8085 *state) {
    return state->memory;
}
UINT8 *getIO(State8085 *state) {
    return state->io;
}

void setSIDLine(State8085 *state, int level) {
    state->sid_line = (level != 0);
}

int getSODLine(State8085 *state) {
    return state->sod_line ? 1 : 0;
}

int triggerInterrupt(State8085 *state, int code, int active) {
    switch (code) {
    case 0: case 1: case 2: case 3:
    case 4: case 5: case 6: case 7:
        // 8080-style INT: external device places RST N on data bus
        state->pending_int = active;
        state->int_rst_number = (UINT8)code;
        break;
    case 45:
        state->pending_trap = active;
        break;
    case 55:
        state->pending_r5 = active;
        break;
    case 65:
        state->pending_r6 = active;
        break;
    case 75:
        if (active == 1) {
            state->r7_latch = 1; // only on rising edge
        }
        break;
    default:
        fprintf(stderr, "Unknown interrupt code: %d\n", code);
        break;
    }
    return state->int_enable;
}
