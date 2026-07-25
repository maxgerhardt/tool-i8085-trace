#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>
#include <stdbool.h>
#include <string.h>

#include "i8085_cpu.h"

// Core derived from sim8085 (BSD 3-Clause). See LICENSE.

int parity(int x, int size) {
    int i;
    int p = 0;
    x = (x & ((1 << size) - 1));
    for (i = 0; i < size; i++) {
        if (x & 0x1)
            p++;
        x = x >> 1;
    }
    return (0 == (p & 0x1));
}

typedef enum { PRESERVE_CARRY, UPDATE_CARRY } should_preserve_carry;

void LogicFlagsA(State8085 *state, uint8_t ac) {
    // Verified in OpenSimH code, that both
    // carry and aux carry are reset.
    state->cc.cy = 0;
    state->cc.ac = ac ? 1 : 0;
    state->cc.z = (state->a == 0);
    state->cc.s = (0x80 == (state->a & 0x80));
    state->cc.p = parity(state->a, 8);
}

void ArithFlagsA(State8085 *state, uint16_t res, should_preserve_carry preserveCarry) {
    if (preserveCarry == UPDATE_CARRY)
        state->cc.cy = (res > 0xff);
    state->cc.z = ((res & 0xff) == 0);
    state->cc.s = (0x80 == (res & 0x80));
    state->cc.p = parity(res & 0xff, 8);
}

void UnimplementedInstruction(State8085 *state) {
    // PC will have advanced one, so undo that
    fprintf(stderr, "Error: Unimplemented instruction\n");
    state->pc--;
    char buf[64];
    Disassemble8085Op(state->memory, state->pc, buf, sizeof(buf));
    fprintf(stderr, "  at 0x%04x: %s\n", state->pc, buf);
    exit(1);
}

void InvalidInstruction(State8085 *state) {
    // pc will have advanced one, so undo that
    fprintf(stderr, "Error: Invalid instruction\n");
    fprintf(stderr, "PC: %u\n", state->pc);
    fprintf(stderr, "Memory at PC: %u\n", state->memory[state->pc]);
    state->pc--;
    exit(1);
}

uint8_t addByte(State8085 *state, uint8_t lhs, uint8_t rhs, should_preserve_carry preserveCarry) {
    uint16_t res = lhs + rhs;
    state->cc.ac = (lhs & 0xf) + (rhs & 0xf) > 0xf;
    // V: overflow when both operands same sign but result differs
    state->cc.v = ((lhs ^ rhs ^ 0x80) & (lhs ^ (uint8_t)res) & 0x80) >> 7;
    ArithFlagsA(state, res, preserveCarry);
    return (uint8_t)res;
}

uint8_t addByteWithCarry(State8085 *state, uint8_t lhs, uint8_t rhs, should_preserve_carry preserveCarry) {
    uint8_t carry = state->cc.cy ? 1 : 0;
    uint16_t res = lhs + rhs + carry;
    state->cc.ac = (lhs & 0xf) + (rhs & 0xf) + carry > 0xf;
    state->cc.v = ((lhs ^ rhs ^ 0x80) & (lhs ^ (uint8_t)res) & 0x80) >> 7;
    ArithFlagsA(state, res, preserveCarry);
    return (uint8_t)res;
}

uint8_t subtractByte(State8085 *state, uint8_t lhs, uint8_t rhs, should_preserve_carry preserveCarry) {
    uint16_t res = lhs - rhs;
    state->cc.ac = (lhs & 0x0f) < (rhs & 0x0f);
    // V: overflow when operands differ in sign and result sign != lhs sign
    state->cc.v = ((lhs ^ rhs) & (lhs ^ (uint8_t)res) & 0x80) >> 7;
    ArithFlagsA(state, res, preserveCarry);
    return (uint8_t)res;
}

uint8_t subtractByteWithBorrow(State8085 *state, uint8_t lhs, uint8_t rhs, should_preserve_carry preserveCarry) {
    uint8_t carry = state->cc.cy ? 1 : 0;
    uint16_t res = lhs - rhs - carry;
    state->cc.ac = (lhs & 0x0f) < ((rhs & 0x0f) + carry);
    state->cc.v = ((lhs ^ rhs) & (lhs ^ (uint8_t)res) & 0x80) >> 7;
    ArithFlagsA(state, res, preserveCarry);
    return (uint8_t)res;
}

void call(State8085 *state, uint16_t addr) {
    uint16_t pc = state->pc + 2;
    state->memory[state->sp - 1] = (pc >> 8) & 0xff;
    state->memory[state->sp - 2] = (pc & 0xff);
    state->sp = state->sp - 2;
    state->pc = addr;
}

void returnToCaller(State8085 *state) {
    state->pc = (state->memory[state->sp] | (state->memory[state->sp + 1] << 8));
    state->sp += 2;
}

void rst(State8085 *state, uint8_t rst_number, uint8_t half) {
    uint16_t pc = state->pc; // PC has already been incremented by Emulate8085Op
    state->memory[state->sp - 1] = (pc >> 8) & 0xff;
    state->memory[state->sp - 2] = (pc & 0xff);
    state->sp = state->sp - 2;
    state->pc = rst_number * 8 + half * 4;
}

bool checkInterrupts(State8085 *state) {
    // Highest priority first
    if (state->pending_trap) {
        state->pending_trap = 0;
        state->int_enable = 0;
        rst(state, 4, 1); // RST 4.5 = 0x24
        return true;
    }

    if (state->int_enable == 0)
        return false;

    if (state->r7_latch && !state->r7_mask) {
        state->r7_latch = 0;
        state->int_enable = 0;
        rst(state, 7, 1); // RST 7.5 = 0x3C
        return true;
    }

    if (state->pending_r6 && !state->r6_mask) {
        state->pending_r6 = 0;
        state->int_enable = 0;
        rst(state, 6, 1); // RST 6.5 = 0x34
        return true;
    }

    if (state->pending_r5 && !state->r5_mask) {
        state->pending_r5 = 0;
        state->int_enable = 0;
        rst(state, 5, 1); // RST 5.5 = 0x2C
        return true;
    }

    // 8080-style INT: external device places RST N on data bus
    if (state->pending_int) {
        UINT8 n = state->int_rst_number;
        state->pending_int = 0;
        state->int_enable = 0;
        rst(state, n, 0); // RST N = N*8 (not half-step)
        return true;
    }

    return false;
}

extern void io_write(int address, int value);
extern void io_pre_read(int address);
extern void io_read(int address, int value);

int Emulate8085Op(State8085 *state, ExecutionStats8085 *stats) {
    bool interruptTaken = checkInterrupts(state);
    if (state->hlt_enable) {
        if (!interruptTaken) {
            return 1;
        }
        state->hlt_enable = 0;
    }

    unsigned char *opcode = &state->memory[state->pc];
    uint8_t current_opcode = *opcode;

    int states = 4; // default fallback

    // Disassemble8085Op(state->memory, state->pc);

    state->pc += 1;

    switch (current_opcode) {
    case 0x00: // NOP
        break;
    case 0x01: // LXI B,word
        state->c = opcode[1];
        state->b = opcode[2];
        state->pc += 2;
        break;
    case 0x02: // STAX B
        state->memory[(state->b << 8) | state->c] = state->a;
        states = 7;
        break;
    case 0x03: // INX B
        state->c++;
        if (state->c == 0)
            state->b++;
        state->cc.x5 = (state->b == 0 && state->c == 0);  // FFFF→0000
        states = 6;
        break;
    case 0x04: // INR B
        state->b = addByte(state, state->b, 1, PRESERVE_CARRY);
        states = 4;
        break;
    case 0x05: // DCR B
        state->b = subtractByte(state, state->b, 1, PRESERVE_CARRY);
        states = 4;
        break;
    case 0x06: // MVI B, byte
        state->b = opcode[1];
        state->pc++;
        states = 10;
        break;
    case 0x07: // RLC
    {
        uint8_t x = state->a;
        state->a = ((x & 0x80) >> 7) | (x << 1);
        state->cc.cy = (1 == ((x & 0x80) >> 7));
        states = 4;
    } break;
    case 0x08: // DSUB (undocumented): HL = HL - BC, all flags
    {
        uint16_t hl = (state->h << 8) | state->l;
        uint16_t bc = (state->b << 8) | state->c;
        uint32_t result = (uint32_t)hl - (uint32_t)bc;
        // V flag: 2's complement overflow
        // Overflow when signs of operands differ and result sign != HL sign
        uint8_t sign_hl = (hl >> 15) & 1;
        uint8_t sign_bc = (bc >> 15) & 1;
        uint8_t sign_res = (result >> 15) & 1;
        state->cc.v = (sign_hl != sign_bc) && (sign_res != sign_hl);
        // Standard flags
        state->cc.cy = (result >> 16) & 1;  // Borrow
        state->cc.z = ((result & 0xffff) == 0);
        state->cc.s = sign_res;
        state->cc.p = parity(result & 0xff, 8);
        state->cc.ac = (hl & 0x0f) < (bc & 0x0f);  // Nibble borrow
        state->h = (result >> 8) & 0xff;
        state->l = result & 0xff;
        states = 10;
    } break;
    case 0x09: // DAD B
    {
        uint32_t hl = (state->h << 8) | state->l;
        uint32_t bc = (state->b << 8) | state->c;
        uint32_t res = hl + bc;
        state->h = (res & 0xff00) >> 8;
        state->l = res & 0xff;
        state->cc.cy = ((res & 0xffff0000) > 0);
        states = 10;
    } break;
    case 0x0a: // LDAX B
    {
        uint16_t offset = (state->b << 8) | state->c;
        state->a = state->memory[offset];
        states = 7;
    } break;
    case 0x0b: // DCX B
        state->c--;
        if (state->c == 0xFF)
            state->b--;
        state->cc.x5 = (state->b == 0xFF && state->c == 0xFF);  // 0000→FFFF
        states = 6;
        break;
    case 0x0c: // INR C
    {
        state->c = addByte(state, state->c, 1, PRESERVE_CARRY);
        states = 4;
    } break;
    case 0x0d: // DCR    C
        state->c = subtractByte(state, state->c, 1, PRESERVE_CARRY);
        states = 4;
        break;
    case 0x0e: // MVI C, byte
        state->c = opcode[1];
        state->pc++;
        states = 10;
        break;
    case 0x0f: // RRC
    {
        uint8_t x = state->a;
        state->a = ((x & 1) << 7) | (x >> 1);
        state->cc.cy = (1 == (x & 1));
        states = 4;
    } break;
    case 0x10: // ARHL (undocumented): Arithmetic right shift HL, CY=L[0]
    {
        uint16_t hl = (state->h << 8) | state->l;
        state->cc.cy = hl & 1;
        hl = (uint16_t)((int16_t)hl >> 1);  // Arithmetic shift preserves sign
        state->h = (hl >> 8) & 0xff;
        state->l = hl & 0xff;
        states = 7;
    } break;
    case 0x11: // LXI	D,word
        state->e = opcode[1];
        state->d = opcode[2];
        state->pc += 2;
        break;
    case 0x12: // STAX D
        state->memory[(state->d << 8) + state->e] = state->a;
        states = 7;
        break;
    case 0x13: // INX    D
        state->e++;
        if (state->e == 0)
            state->d++;
        state->cc.x5 = (state->d == 0 && state->e == 0);  // FFFF→0000
        states = 6;
        break;
    case 0x14: // INR D
        state->d = addByte(state, state->d, 1, PRESERVE_CARRY);
        states = 4;
        break;
    case 0x15: // DCR D
        state->d = subtractByte(state, state->d, 1, PRESERVE_CARRY);
        states = 4;
        break;
    case 0x16: // MVI D, byte
        state->d = opcode[1];
        state->pc++;
        states = 10;
        break;
    case 0x17: // RAL
    {
        uint8_t x = state->a;
        state->a = state->cc.cy | (x << 1);
        state->cc.cy = (1 == ((x & 0x80) >> 7));
        states = 4;
    } break;
    case 0x18: // RDEL (undocumented): Rotate DE left through carry
    {
        uint16_t de = (state->d << 8) | state->e;
        uint8_t old_cy = state->cc.cy;
        state->cc.cy = (de >> 15) & 1;  // D7 goes to carry
        de = (de << 1) | old_cy;        // Old carry goes to E0
        // V flag: overflow if sign changed
        state->cc.v = state->cc.cy ^ ((de >> 15) & 1);
        state->d = (de >> 8) & 0xff;
        state->e = de & 0xff;
        states = 10;
    } break;
    case 0x19: // DAD D
    {
        uint32_t hl = (state->h << 8) | state->l;
        uint32_t de = (state->d << 8) | state->e;
        uint32_t res = hl + de;
        state->h = (res & 0xff00) >> 8;
        state->l = res & 0xff;
        state->cc.cy = ((res & 0xffff0000) != 0);
        states = 10;
    } break;
    case 0x1a: // LDAX D
    {
        uint16_t offset = (state->d << 8) | state->e;
        state->a = state->memory[offset];
        states = 7;
    } break;
    case 0x1b: // DCX D
        state->e--;
        if (state->e == 0xFF)
            state->d--;
        state->cc.x5 = (state->d == 0xFF && state->e == 0xFF);  // 0000→FFFF
        states = 6;
        break;
    case 0x1c: // INR E
        state->e = addByte(state, state->e, 1, PRESERVE_CARRY);
        states = 4;
        break;
    case 0x1d: // DCR E
        state->e = subtractByte(state, state->e, 1, PRESERVE_CARRY);
        states = 4;
        break;
    case 0x1e: // MVI E, byte
        state->e = opcode[1];
        state->pc++;
        states = 10;
        break;
    case 0x1f: // RAR
    {
        uint8_t x = state->a;
        state->a = (x >> 1) | (state->cc.cy << 7); /* From a number with higest bit as carry value */
        state->cc.cy = (1 == (x & 1));
        states = 4;
    } break;
    case 0x20: // RIM
    {
        uint8_t result = 0;

        result |= (state->sid_line ? 1 : 0) << 7;
        result |= (state->r7_latch ? 1 : 0) << 6;
        result |= (state->pending_r6 ? 1 : 0) << 5;
        result |= (state->pending_r5 ? 1 : 0) << 4;
        result |= (state->int_enable ? 1 : 0) << 3;
        result |= (state->r7_mask ? 1 : 0) << 2;
        result |= (state->r6_mask ? 1 : 0) << 1;
        result |= (state->r5_mask ? 1 : 0) << 0;

        state->a = result;
        states = 4;
    } break;
    case 0x21: // LXI H,word
        state->l = opcode[1];
        state->h = opcode[2];
        state->pc += 2;
        break;
    case 0x22: // SHLD word
    {
        uint16_t offset = (opcode[2] << 8) | opcode[1];
        state->memory[offset] = state->l;
        state->memory[offset + 1] = state->h;
        state->pc += 2;
        states = 16;
    } break;
    case 0x23: // INX H
        state->l++;
        if (state->l == 0)
            state->h++;
        state->cc.x5 = (state->h == 0 && state->l == 0);  // FFFF→0000
        states = 6;
        break;
    case 0x24: // INR H
        state->h = addByte(state, state->h, 1, PRESERVE_CARRY);
        states = 4;
        break;
        break;
    case 0x25: // DCR H
        state->h = subtractByte(state, state->h, 1, PRESERVE_CARRY);
        states = 4;
        break;
    case 0x26: // MVI H, byte
        state->h = opcode[1];
        state->pc++;
        states = 10;
        break;
    case 0x27: // DAA
    {
        uint8_t a = state->a;
        uint8_t correction = 0;
        uint8_t cy = state->cc.cy;

        if ((a & 0x0f) > 9 || state->cc.ac) {
            correction |= 0x06;
        }

        if (a > 0x99 || state->cc.cy) {
            correction |= 0x60;
            cy = 1;
        }

        uint16_t res = (uint16_t)a + correction;
        state->cc.cy = cy;
        state->cc.ac = ((a & 0x0f) + (correction & 0x0f)) > 0x0f;
        state->a = (uint8_t)res;
        state->cc.z = (state->a == 0);
        state->cc.s = (0x80 == (state->a & 0x80));
        state->cc.p = parity(state->a, 8);
        states = 4;
    } break;
    case 0x28: // LDHI (undocumented): DE = HL + imm8, no flags
    {
        uint16_t hl = (state->h << 8) | state->l;
        uint16_t result = hl + opcode[1];
        state->d = (result >> 8) & 0xff;
        state->e = result & 0xff;
        state->pc++;
        states = 10;
    } break;
    case 0x29: // DAD H
    {
        uint32_t hl = (state->h << 8) | state->l;
        uint32_t res = hl + hl;
        state->h = (res & 0xff00) >> 8;
        state->l = res & 0xff;
        state->cc.cy = ((res & 0xffff0000) != 0);
        states = 10;
    } break;
    case 0x2a: // LHLD Addr
    {
        uint16_t offset = (opcode[2] << 8) | (opcode[1]);
        uint8_t l = state->memory[offset];
        uint8_t h = state->memory[offset + 1];
        uint16_t v = h << 8 | l;
        state->h = v >> 8 & 0xFF;
        state->l = v & 0xFF;
        state->pc += 2;
        states = 16;
    } break;
    case 0x2b: // DCX H
        state->l--;
        if (state->l == 0xFF)
            state->h--;
        state->cc.x5 = (state->h == 0xFF && state->l == 0xFF);  // 0000→FFFF
        states = 6;
        break;
    case 0x2c: // INR L
        state->l = addByte(state, state->l, 1, PRESERVE_CARRY);
        states = 4;
        break;
        break;
    case 0x2d: // DCR L
        state->l = subtractByte(state, state->l, 1, PRESERVE_CARRY);
        states = 4;
        break;
    case 0x2e: // MVI L,byte
        state->l = opcode[1];
        state->pc++;
        states = 10;
        break;
    case 0x2f: // CMA
        state->a ^= 0xFF;
        states = 4;
        break;
    case 0x30: // SIM
    {
        uint8_t acc = state->a;

        // Bit 6: SDE (Serial Data Enable)
        // Bit 7: SOD (Serial Output Data)
        if ((acc & 0x40) != 0) { // SDE bit set
            uint8_t sod = (acc & 0x80) ? 1 : 0;
            state->sod_line = sod;
            // TODO Notify on SOD line
        }

        // Bit 4: Reset RST 7.5 latch (edge-triggered flip-flop)
        if (acc & 0x10) {
            state->r7_latch = 0; // Clear the latched interrupt
        }

        // Bit 3: MSE (Mask Set Enable)
        if (acc & 0x08) {
            state->r7_mask = (acc >> 2) & 1; // Bit 2 → RST 7.5
            state->r6_mask = (acc >> 1) & 1; // Bit 1 → RST 6.5
            state->r5_mask = (acc >> 0) & 1; // Bit 0 → RST 5.5
        }

        states = 4;
    } break;
    case 0x31: // LXI SP, word
        state->sp = (opcode[2] << 8) | opcode[1];
        state->pc += 2;
        states = 10;
        break;
    case 0x32: // STA word
    {
        uint16_t offset = (opcode[2] << 8) | (opcode[1]);
        state->memory[offset] = state->a;
        state->pc += 2;
        states = 13;
    } break;
    case 0x33: // INX SP
        state->sp++;
        state->cc.x5 = (state->sp == 0);  // FFFF→0000
        states = 6;
        break;
    case 0x34: // INR M
    {
        uint16_t offset = (state->h << 8) | state->l;
        state->memory[offset] = addByte(state, state->memory[offset], 1, PRESERVE_CARRY);
        states = 10;
    } break;
    case 0x35: // DCR M
    {
        uint16_t offset = (state->h << 8) | state->l;
        state->memory[offset] = subtractByte(state, state->memory[offset], 1, PRESERVE_CARRY);
        states = 10;
    } break;
    case 0x36: // MVI M, byte
    {
        // AC set if lower nibble of h was zero prior to dec
        uint16_t offset = (state->h << 8) | state->l;
        state->memory[offset] = opcode[1];
        state->pc++;
        states = 10;
    } break;
    case 0x37:
        state->cc.cy = 1;
        states = 4;
        break; // STC
    case 0x38: // LDSI (undocumented): DE = SP + imm8, no flags
    {
        uint16_t result = state->sp + opcode[1];
        state->d = (result >> 8) & 0xff;
        state->e = result & 0xff;
        state->pc++;
        states = 10;
    } break;
    case 0x39: // DAD SP
    {
        uint16_t hl = (state->h << 8) | state->l;
        uint16_t sp = state->sp;
        uint32_t res = hl + sp;
        state->h = (res & 0xff00) >> 8;
        state->l = res & 0xff;
        state->cc.cy = ((res & 0xffff0000) > 0);
        states = 10;
    } break;
        break;
    case 0x3a: // LDA word
    {
        uint16_t offset = (opcode[2] << 8) | (opcode[1]);
        state->a = state->memory[offset];
        state->pc += 2;
        states = 13;
    } break;
    case 0x3b: // DCX SP
        state->sp--;
        state->cc.x5 = (state->sp == 0xFFFF);  // 0000→FFFF
        states = 6;
        break;
    case 0x3c: // INR A
        state->a = addByte(state, state->a, 1, PRESERVE_CARRY);
        states = 4;
        break;
    case 0x3d: // DCR A
        state->a = subtractByte(state, state->a, 1, PRESERVE_CARRY);
        states = 4;
        break;
    case 0x3e: // MVI A, byte
        state->a = opcode[1];
        state->pc++;
        states = 10;
        break;
    case 0x3f: // CMC
        if (0 == state->cc.cy)
            state->cc.cy = 1;
        else
            state->cc.cy = 0;
        states = 4;
        break;
    case 0x40:
        state->b = state->b;
        states = 4;
        break; // MOV B, B
    case 0x41:
        state->b = state->c;
        states = 4;
        break; // MOV B, C
    case 0x42:
        state->b = state->d;
        states = 4;
        break; // MOV B, D
    case 0x43:
        state->b = state->e;
        states = 4;
        break; // MOV B, E
    case 0x44:
        state->b = state->h;
        states = 4;
        break; // MOV B, H
    case 0x45:
        state->b = state->l;
        states = 4;
        break; // MOV B, L
    case 0x46: // MOV B, M
    {
        uint16_t offset = (state->h << 8) | (state->l);
        state->b = state->memory[offset];
        states = 7;
    } break;
    case 0x47:
        state->b = state->a;
        states = 4;
        break; // MOV B, A
    case 0x48:
        state->c = state->b;
        states = 4;
        break; // MOV C, B
    case 0x49:
        state->c = state->c;
        states = 4;
        break; // MOV C, C
    case 0x4a:
        state->c = state->d;
        states = 4;
        break; // MOV C, D
    case 0x4b:
        state->c = state->e;
        states = 4;
        break; // MOV C, E
    case 0x4c:
        state->c = state->h;
        states = 4;
        break; // MOV C, H
    case 0x4d:
        state->c = state->l;
        states = 4;
        break; // MOV C, L
    case 0x4e: // MOV C, M
    {
        uint16_t offset = (state->h << 8) | (state->l);
        state->c = state->memory[offset];
        states = 7;
    } break;
    case 0x4f:
        state->c = state->a;
        states = 4;
        break; // MOV C, A
    case 0x50:
        state->d = state->b;
        states = 4;
        break; // MOV D, B
    case 0x51: // MOV D, C
        state->d = state->c;
        states = 4;
        break;
    case 0x52: // MOV D, D
        state->d = state->d;
        states = 4;
        break;
    case 0x53: // MOV D, E
        state->d = state->e;
        states = 4;
        break;
    case 0x54:
        state->d = state->h;
        states = 4;
        break; // MOV D, H
    case 0x55:
        state->d = state->l;
        states = 4;
        break; // MOV D, B
    case 0x56: // MOV D, M
    {
        uint16_t offset = (state->h << 8) | (state->l);
        state->d = state->memory[offset];
        states = 7;
    } break;
    case 0x57:
        state->d = state->a;
        states = 4;
        break; // MOV D, A
    case 0x58:
        state->e = state->b;
        states = 4;
        break; // MOV E, B
    case 0x59:
        state->e = state->c;
        states = 4;
        break; // MOV E, C
    case 0x5a:
        state->e = state->d;
        states = 4;
        break; // MOV E, D
    case 0x5b:
        state->e = state->e;
        states = 4;
        break; // MOV E, E
    case 0x5c:
        state->e = state->h;
        states = 4;
        break; // MOV E, H
    case 0x5d:
        state->e = state->l;
        states = 4;
        break; // MOV E, L
    case 0x5e: // MOV E, M
    {
        uint16_t offset = (state->h << 8) | (state->l);
        state->e = state->memory[offset];
        states = 7;
    } break;
    case 0x5f:
        state->e = state->a;
        states = 4;
        break; // MOV E, A
    case 0x60:
        state->h = state->b;
        states = 4;
        break; // MOV H, B
    case 0x61:
        state->h = state->c;
        states = 4;
        break; // MOV H, C
    case 0x62:
        state->h = state->d;
        states = 4;
        break; // MOV H, D
    case 0x63:
        state->h = state->e;
        states = 4;
        break; // MOV H, E
    case 0x64:
        state->h = state->h;
        states = 4;
        break; // MOV H, H
    case 0x65:
        state->h = state->l;
        states = 4;
        break; // MOV H, L
    case 0x66: // MOV H, M
    {
        uint16_t offset = (state->h << 8) | (state->l);
        state->h = state->memory[offset];
        states = 7;
    } break;
    case 0x67:
        state->h = state->a;
        states = 4;
        break; // MOV H, A
    case 0x68:
        state->l = state->b;
        states = 4;
        break; // MOV L, B
    case 0x69:
        state->l = state->c;
        states = 4;
        break; // MOV L, C
    case 0x6a:
        state->l = state->d;
        states = 4;
        break; // MOV L, D
    case 0x6b:
        state->l = state->e;
        states = 4;
        break; // MOV L, E
    case 0x6c:
        state->l = state->h;
        states = 4;
        break; // MOV L, H
    case 0x6d:
        state->l = state->l;
        states = 4;
        break; // MOV L, L
    case 0x6e: // MOV L, M
    {
        uint16_t offset = (state->h << 8) | (state->l);
        state->l = state->memory[offset];
        states = 7;
    } break;
    case 0x6f:
        state->l = state->a;
        states = 4;
        break; // MOV L, A
    case 0x70: // MOV M, B
    {
        uint16_t offset = (state->h << 8) | (state->l);
        state->memory[offset] = state->b;
        states = 7;
    } break;
    case 0x71: // MOV M, C
    {
        uint16_t offset = (state->h << 8) | (state->l);
        state->memory[offset] = state->c;
        states = 7;
    } break;
    case 0x72: // MOV M, D
    {
        uint16_t offset = (state->h << 8) | (state->l);
        state->memory[offset] = state->d;
        states = 7;
    } break;
    case 0x73: // MOV M, E
    {
        uint16_t offset = (state->h << 8) | (state->l);
        state->memory[offset] = state->e;
        states = 7;
    } break;
    case 0x74: // MOV M, H
    {
        uint16_t offset = (state->h << 8) | (state->l);
        state->memory[offset] = state->h;
        states = 7;
    } break;
    case 0x75: // MOV M, L
    {
        uint16_t offset = (state->h << 8) | (state->l);
        state->memory[offset] = state->l;
        states = 7;
    } break;
    case 0x76: // HLT
        state->hlt_enable = 1;
        states = 5;
        break;
    case 0x77: // MOV M, A
    {
        uint16_t offset = (state->h << 8) | (state->l);
        state->memory[offset] = state->a;
        states = 7;
    } break;
    case 0x78:
        state->a = state->b;
        states = 4;
        break; // MOV A, B
    case 0x79:
        state->a = state->c;
        states = 4;
        break; // MOV A, C
    case 0x7a:
        state->a = state->d;
        states = 4;
        break; // MOV A, D
    case 0x7b:
        state->a = state->e;
        states = 4;
        break; // MOV A, E
    case 0x7c:
        state->a = state->h;
        states = 4;
        break; // MOV A, H
    case 0x7d:
        state->a = state->l;
        states = 4;
        break; // MOV A, L
    case 0x7e: // MOV A, M
    {
        uint16_t offset = (state->h << 8) | (state->l);
        state->a = state->memory[offset];
        states = 7;
    } break;
    case 0x7f:
        state->a = state->a;
        states = 4;
        break; // MOV A, A
    case 0x80: // ADD B
        state->a = addByte(state, state->a, state->b, UPDATE_CARRY);
        states = 4;
        break;
    case 0x81: // ADD C
        state->a = addByte(state, state->a, state->c, UPDATE_CARRY);
        states = 4;
        break;
    case 0x82: // ADD D
        state->a = addByte(state, state->a, state->d, UPDATE_CARRY);
        states = 4;
        break;
    case 0x83: // ADD E
        state->a = addByte(state, state->a, state->e, UPDATE_CARRY);
        states = 4;
        break;
    case 0x84: // ADD H
        state->a = addByte(state, state->a, state->h, UPDATE_CARRY);
        states = 4;
        break;
    case 0x85: // ADD L
        state->a = addByte(state, state->a, state->l, UPDATE_CARRY);
        states = 4;
        break;
    case 0x86: // ADD M
    {
        uint16_t offset = (state->h << 8) | (state->l);
        state->a = addByte(state, state->a, state->memory[offset], UPDATE_CARRY);
        states = 7;
    } break;
    case 0x87: // ADD A
        state->a = addByte(state, state->a, state->a, UPDATE_CARRY);
        states = 4;
        break;
    case 0x88: // ADC B
        state->a = addByteWithCarry(state, state->a, state->b, UPDATE_CARRY);
        states = 4;
        break;
    case 0x89: // ADC C
        state->a = addByteWithCarry(state, state->a, state->c, UPDATE_CARRY);
        states = 4;
        break;
        break;
    case 0x8a: // ADC D
        state->a = addByteWithCarry(state, state->a, state->d, UPDATE_CARRY);
        states = 4;
        break;
    case 0x8b: // ADC E
        state->a = addByteWithCarry(state, state->a, state->e, UPDATE_CARRY);
        states = 4;
        break;
    case 0x8c: // ADC H
        state->a = addByteWithCarry(state, state->a, state->h, UPDATE_CARRY);
        states = 4;
        break;
    case 0x8d: // ADC L
        state->a = addByteWithCarry(state, state->a, state->l, UPDATE_CARRY);
        states = 4;
        break;
    case 0x8e: // ADC M
    {
        uint16_t offset = (state->h << 8) | (state->l);
        state->a = addByteWithCarry(state, state->a, state->memory[offset], UPDATE_CARRY);
        states = 7;
    } break;
    case 0x8f: // ADC A
        state->a = addByteWithCarry(state, state->a, state->a, UPDATE_CARRY);
        states = 4;
        break;
    case 0x90: // SUB B
        state->a = subtractByte(state, state->a, state->b, UPDATE_CARRY);
        break;
    case 0x91: // SUB C
        state->a = subtractByte(state, state->a, state->c, UPDATE_CARRY);
        break;
    case 0x92: // SUB D
        state->a = subtractByte(state, state->a, state->d, UPDATE_CARRY);
        break;
    case 0x93: // SUB E
        state->a = subtractByte(state, state->a, state->e, UPDATE_CARRY);
        break;
    case 0x94: // SUB H
        state->a = subtractByte(state, state->a, state->h, UPDATE_CARRY);
        break;
    case 0x95: // SUB L
        state->a = subtractByte(state, state->a, state->l, UPDATE_CARRY);
        break;
    case 0x96: // SUB M
    {
        uint16_t offset = (state->h << 8) | (state->l);
        state->a = subtractByte(state, state->a, state->memory[offset], UPDATE_CARRY);
        states = 7;
    } break;
    case 0x97: // SUB A
        state->a = subtractByte(state, state->a, state->a, UPDATE_CARRY);
        break;
    case 0x98: // SBB B
        state->a = subtractByteWithBorrow(state, state->a, state->b, UPDATE_CARRY);
        states = 4;
        break;
    case 0x99: // SBB C
        state->a = subtractByteWithBorrow(state, state->a, state->c, UPDATE_CARRY);
        states = 4;
        break;
    case 0x9a: // SBB D
        state->a = subtractByteWithBorrow(state, state->a, state->d, UPDATE_CARRY);
        states = 4;
        break;
    case 0x9b: // SBB E
        state->a = subtractByteWithBorrow(state, state->a, state->e, UPDATE_CARRY);
        states = 4;
        break;
    case 0x9c: // SBB H
        state->a = subtractByteWithBorrow(state, state->a, state->h, UPDATE_CARRY);
        states = 4;
        break;
    case 0x9d: // SBB L
        state->a = subtractByteWithBorrow(state, state->a, state->l, UPDATE_CARRY);
        states = 4;
        break;
    case 0x9e: // SBB M
    {
        uint16_t offset = (state->h << 8) | (state->l);
        state->a = subtractByteWithBorrow(state, state->a, state->memory[offset], UPDATE_CARRY);
        states = 7;
    } break;
    case 0x9f: // SBB A
        state->a = subtractByteWithBorrow(state, state->a, state->a, UPDATE_CARRY);
        states = 4;
        break;
    case 0xa0: // ANA B
        state->a = state->a & state->b;
        LogicFlagsA(state, 1);
        states = 4;
        break;
    case 0xa1: // ANA C
        state->a = state->a & state->c;
        LogicFlagsA(state, 1);
        states = 4;
        break;
    case 0xa2: // ANA D
        state->a = state->a & state->d;
        LogicFlagsA(state, 1);
        states = 4;
        break;
    case 0xa3: // ANA E
        state->a = state->a & state->e;
        LogicFlagsA(state, 1);
        states = 4;
        break;
    case 0xa4: // ANA H
        state->a = state->a & state->h;
        LogicFlagsA(state, 1);
        states = 4;
        break;
    case 0xa5: // ANA L
        state->a = state->a & state->l;
        LogicFlagsA(state, 1);
        states = 4;
        break;
    case 0xa6: // ANA M
    {
        uint16_t offset = (state->h << 8) | (state->l);
        state->a = state->a & state->memory[offset];
        LogicFlagsA(state, 1);
        states = 7;
    } break;
    case 0xa7: // ANA A
        state->a = state->a & state->a;
        LogicFlagsA(state, 1);
        states = 4;
        break;
    case 0xa8:
        state->a = state->a ^ state->b;
        LogicFlagsA(state, 0);
        break; // XRA B
    case 0xa9:
        state->a = state->a ^ state->c;
        LogicFlagsA(state, 0);
        break; // XRA C
    case 0xaa:
        state->a = state->a ^ state->d;
        LogicFlagsA(state, 0);
        break; // XRA D
    case 0xab:
        state->a = state->a ^ state->e;
        LogicFlagsA(state, 0);
        break; // XRA E
    case 0xac:
        state->a = state->a ^ state->h;
        LogicFlagsA(state, 0);
        break; // XRA H
    case 0xad:
        state->a = state->a ^ state->l;
        LogicFlagsA(state, 0);
        break; // XRA L
    case 0xae: // XRA M
    {
        uint16_t offset = (state->h << 8) | (state->l);
        state->a = state->a ^ state->memory[offset];
        LogicFlagsA(state, 0);
        states = 7;
    } break;
    case 0xaf:
        state->a = state->a ^ state->a;
        LogicFlagsA(state, 0);
        break; // XRA A
    case 0xb0:
        state->a = state->a | state->b;
        LogicFlagsA(state, 0);
        states = 4;
        break; // ORA B
    case 0xb1:
        state->a = state->a | state->c;
        LogicFlagsA(state, 0);
        states = 4;
        break; // ORA C
    case 0xb2:
        state->a = state->a | state->d;
        LogicFlagsA(state, 0);
        states = 4;
        break; // ORA D
    case 0xb3:
        state->a = state->a | state->e;
        LogicFlagsA(state, 0);
        states = 4;
        break; // ORA E
    case 0xb4:
        state->a = state->a | state->h;
        LogicFlagsA(state, 0);
        states = 4;
        break; // ORA H
    case 0xb5: // ORA L
        state->a = state->a | state->l;
        LogicFlagsA(state, 0);
        states = 4;
        break;
    case 0xb6: // ORA M
    {
        uint16_t offset = (state->h << 8) | (state->l);
        state->a = state->a | state->memory[offset];
        LogicFlagsA(state, 0);
        states = 7;
    } break;
    case 0xb7: // ORA A
        state->a = state->a | state->a;
        LogicFlagsA(state, 0);
        states = 4;
        break;
    case 0xb8: // CMP B
        subtractByte(state, state->a, state->b, UPDATE_CARRY);
        states = 4;
        break;
    case 0xb9: // CMP C
        subtractByte(state, state->a, state->c, UPDATE_CARRY);
        states = 4;
        break;
    case 0xba: // CMP D
        subtractByte(state, state->a, state->d, UPDATE_CARRY);
        states = 4;
        break;
    case 0xbb: // CMP E
        subtractByte(state, state->a, state->e, UPDATE_CARRY);
        states = 4;
        break;
    case 0xbc: // CMP H
        subtractByte(state, state->a, state->h, UPDATE_CARRY);
        states = 4;
        break;
    case 0xbd: // CMP L
        subtractByte(state, state->a, state->l, UPDATE_CARRY);
        states = 4;
        break;
    case 0xbe: // CMP M
    {
        uint16_t offset = (state->h << 8) | (state->l);
        subtractByte(state, state->a, state->memory[offset], UPDATE_CARRY);
        states = 7;
    } break;
    case 0xbf: // CMP A
        subtractByte(state, state->a, state->a, UPDATE_CARRY);
        states = 4;
        break;
    case 0xc0: // RNZ
        states = 6;
        if (0 == state->cc.z) {
            states = 12;
            returnToCaller(state);
        }
        break;
    case 0xc1: // POP B
    {
        state->c = state->memory[state->sp];
        state->b = state->memory[state->sp + 1];
        state->sp += 2;
        states = 10;
    } break;
    case 0xc2: // JNZ Addr
        if (0 == state->cc.z) {
            state->pc = ((opcode[2] << 8) | opcode[1]);
            states = 10;
        } else {
            state->pc += 2;
            states = 7;
        }
        break;
    case 0xc3: // JMP Addr
        state->pc = ((opcode[2] << 8) | opcode[1]);
        states = 10;
        break;
    case 0xc4: // CNZ Addr
        if (0 == state->cc.z) {
            call(state, (opcode[2] << 8) | opcode[1]);
            states = 18;
        } else
            state->pc += 2;
        states = 9;
        break;
    case 0xc5: // PUSH   B
    {
        state->memory[state->sp - 1] = state->b;
        state->memory[state->sp - 2] = state->c;
        state->sp = state->sp - 2;
        states = 13;
    } break;
    case 0xc6: // ADI byte
    {
        uint16_t x = (uint16_t)state->a + (uint16_t)opcode[1];
        state->cc.z = ((x & 0xff) == 0);
        state->cc.s = (0x80 == (x & 0x80));
        state->cc.p = parity((x & 0xff), 8);
        state->cc.cy = (x > 0xff);
        state->cc.ac = (((state->a & 0x0f) + (opcode[1] & 0x0f)) > 0x0f);
        state->a = (uint8_t)x;
        state->pc++;
        states = 7;
    } break;
    case 0xc7: // RST 0
        rst(state, 0, 0);
        states = 12;
        break;
    case 0xc8: // RZ
        states = 6;
        if (1 == state->cc.z) {
            states = 12;
            returnToCaller(state);
        }
        break;
    case 0xc9: // RET
        returnToCaller(state);
        states = 10;
        break;
    case 0xca: // JZ Addr
        if (1 == state->cc.z) {
            state->pc = ((opcode[2] << 8) | opcode[1]);
            states = 10;
        } else {
            state->pc += 2;
            states = 7;
        }
        break;
    case 0xcb: // RSTV (undocumented): RST 0x40 if V flag set, else NOP
        if (state->cc.v) {
            rst(state, 0, 0);   // Push PC, but we override target
            state->pc = 0x0040; // Override RST 0 target to 0x40
            states = 12;
        } else {
            states = 6;
        }
        break;
    case 0xcc: // CZ Addr
        if (1 == state->cc.z) {
            call(state, (opcode[2] << 8) | opcode[1]);
            states = 18;
        } else {
            state->pc += 2;
            states = 9;
        }
        break;
    case 0xcd: // CALL Addr
        call(state, (opcode[2] << 8) | opcode[1]);
        states = 18;
        break;
    case 0xce: // ACI d8
        state->a = addByteWithCarry(state, state->a, opcode[1], UPDATE_CARRY);
        state->pc++;
        states = 7;
        break;
    case 0xcf: // RST 1
        rst(state, 1, 0);
        states = 12;
        break;
    case 0xd0: // RNC
        states = 6;
        if (0 == state->cc.cy) {
            returnToCaller(state);
            states = 12;
        }
        break;
    case 0xd1: // POP D
    {
        state->e = state->memory[state->sp];
        state->d = state->memory[state->sp + 1];
        state->sp += 2;
        states = 10;
    } break;
    case 0xd2: // JNC Addr
        if (0 == state->cc.cy) {
            state->pc = ((opcode[2] << 8) | opcode[1]);
            states = 10;
        } else {
            state->pc += 2;
            states = 7;
        }
        break;
    case 0xd3: // OUT d8
        state->io[opcode[1]] = state->a;
        state->pc += 1;
        states = 10;
        io_write(opcode[1], state->a);
        break;
    case 0xd4: // CNC Addr
        if (0 == state->cc.cy) {
            call(state, (opcode[2] << 8) | opcode[1]);
            states = 18;
        } else {
            state->pc += 2;
            states = 9;
        }
        break;
    case 0xd5: // PUSH   D
    {
        state->memory[state->sp - 1] = state->d;
        state->memory[state->sp - 2] = state->e;
        state->sp = state->sp - 2;
        states = 13;
    } break;
    case 0xd6: // SUI d8
        state->a = subtractByte(state, state->a, opcode[1], UPDATE_CARRY);
        state->pc++;
        states = 7;
        break;
    case 0xd7: // RST 2
        rst(state, 2, 0);
        states = 12;
        break;
    case 0xd8: // RC
        states = 6;
        if (1 == state->cc.cy) {
            states = 12;
            returnToCaller(state);
        }
        break;
    case 0xd9: // SHLX (undocumented): [DE] = L, [DE+1] = H
    {
        uint16_t de = (state->d << 8) | state->e;
        state->memory[de] = state->l;
        state->memory[de + 1] = state->h;
        states = 10;
    } break;
    case 0xda: // JC Addr
        if (1 == state->cc.cy) {
            state->pc = ((opcode[2] << 8) | opcode[1]);
            states = 10;
        } else {
            state->pc += 2;
            states = 7;
        }
        break;
    case 0xdb: // IN d8
        io_pre_read(opcode[1]);
        state->a = state->io[opcode[1]];
        state->pc++;
        states = 10;
        io_read(opcode[1], state->a);
        break;
    case 0xdc: // CC Addr
        if (1 == state->cc.cy) {
            call(state, (opcode[2] << 8) | opcode[1]);
            states = 18;
        } else {
            state->pc += 2;
            states = 9;
        }
        break;
    case 0xdd: // JNX5 (undocumented): Jump if not X5
        if (0 == state->cc.x5) {
            state->pc = ((opcode[2] << 8) | opcode[1]);
            states = 10;
        } else {
            state->pc += 2;
            states = 7;
        }
        break;
    case 0xde: // SBI d8
        state->a = subtractByteWithBorrow(state, state->a, opcode[1], UPDATE_CARRY);
        state->pc++;
        states = 7;
        break;
    case 0xdf: // RST 3
        rst(state, 3, 0);
        states = 12;
        break;
    case 0xe0: // RPO
        states = 6;
        if (0 == state->cc.p) {
            states = 12;
            returnToCaller(state);
        }
        break;
    case 0xe1: // POP H
    {
        state->l = state->memory[state->sp];
        state->h = state->memory[state->sp + 1];
        state->sp += 2;
        states = 10;
    } break;
    case 0xe2: // JPO Addr
        if (0 == state->cc.p) {
            state->pc = ((opcode[2] << 8) | opcode[1]);
            states = 10;
        } else {
            state->pc += 2;
            states = 7;
        }
        break;
    case 0xe3: // XTHL
    {
        uint16_t spL = state->memory[state->sp];
        uint16_t spH = state->memory[state->sp + 1];
        state->memory[state->sp] = state->l;
        state->memory[state->sp + 1] = state->h;
        state->h = spH;
        state->l = spL;
        states = 16;
    } break;
    case 0xe4: // CPO Addr
        if (0 == state->cc.p) {
            call(state, (opcode[2] << 8) | opcode[1]);
            states = 18;
        } else {
            state->pc += 2;
            states = 9;
        }
        break;
    case 0xe5: // PUSH H
    {
        state->memory[state->sp - 1] = state->h;
        state->memory[state->sp - 2] = state->l;
        state->sp = state->sp - 2;
        states = 13;
    } break;
    case 0xe6: // ANI byte
    {
        state->a = state->a & opcode[1];
        LogicFlagsA(state, 1);
        state->pc++;
        states = 7;
    } break;
    case 0xe7: // RST 4
        rst(state, 4, 0);
        states = 12;
        break;
    case 0xe8: // RPE
        states = 6;
        if (1 == state->cc.p) {
            states = 12;
            returnToCaller(state);
        }
        break;
    case 0xe9: // PCHL
        state->pc = (state->h << 8) | state->l;
        states = 6;
        break;
    case 0xea: // JPE Addr
        if (1 == state->cc.p) {
            state->pc = ((opcode[2] << 8) | opcode[1]);
            states = 10;
        } else {
            state->pc += 2;
            states = 7;
        }
        break;
    case 0xeb: // XCHG
    {
        uint8_t save1 = state->d;
        uint8_t save2 = state->e;
        state->d = state->h;
        state->e = state->l;
        state->h = save1;
        state->l = save2;
        states = 4;
    } break;
    case 0xec: // CPE Addr
        if (1 == state->cc.p) {
            call(state, (opcode[2] << 8) | opcode[1]);
            states = 18;
        } else {
            state->pc += 2;
            states = 9;
        }
        break;
    case 0xed: // LHLX (undocumented): L = [DE], H = [DE+1]
    {
        uint16_t de = (state->d << 8) | state->e;
        state->l = state->memory[de];
        state->h = state->memory[de + 1];
        states = 10;
    } break;
    case 0xee: // XRI d8
        state->a = state->a ^ opcode[1];
        LogicFlagsA(state, 0);
        state->pc++;
        states = 7;
        break;
    case 0xef: // RST 5
        rst(state, 5, 0);
        states = 12;
        break;
    case 0xf0: // RP
        states = 6;
        if (0 == state->cc.s) {
            states = 12;
            returnToCaller(state);
        }
        break;
    case 0xf1: // POP PSW
    {
        // Step 1: Restore the condition flags from the current stack pointer location
        uint8_t psw = state->memory[state->sp];

        // Step 2: Extract the condition flags from the PSW byte
        state->cc.cy = (psw & 0x01);       // Carry flag (bit 0)
        state->cc.v = (psw & 0x02) >> 1;   // V overflow flag (bit 1, undocumented)
        state->cc.p = (psw & 0x04) >> 2;   // Parity flag (bit 2)
        state->cc.ac = (psw & 0x10) >> 4;  // Auxiliary carry flag (bit 4)
        state->cc.x5 = (psw & 0x20) >> 5;  // X5 flag (bit 5, undocumented)
        state->cc.z = (psw & 0x40) >> 6;   // Zero flag (bit 6)
        state->cc.s = (psw & 0x80) >> 7;   // Sign flag (bit 7)

        // Step 3: Increment the stack pointer to the next memory location
        state->sp++;

        // Step 4: Restore the accumulator from the new stack pointer location
        state->a = state->memory[state->sp];

        // Step 5: Increment the stack pointer again
        state->sp++;

        states = 10;
    } break;
    case 0xf2: // JP Addr
        if (0 == state->cc.s) {
            state->pc = ((opcode[2] << 8) | opcode[1]);
            states = 10;
        } else {
            state->pc += 2;
            states = 7;
        }
        break;
    case 0xf3: // DI
        state->int_enable = 0;
        state->int_enable_delay = 0;
        states = 4;
        break;
    case 0xf4: // CP Addr
        if (0 == state->cc.s) {
            call(state, (opcode[2] << 8) | opcode[1]);
            states = 18;
        } else {
            state->pc += 2;
            states = 9;
        }
        break;
    case 0xf5: // PUSH PSW
    {
        // Step 1: Decrement the stack pointer
        state->sp--;

        // Step 2: Store the accumulator at the new stack pointer location
        state->memory[state->sp] = state->a;

        // Step 3: Decrement the stack pointer again
        state->sp--;

        // Step 4: Construct the PSW byte (format: s z x5 ac 0 p v cy)
        uint8_t psw = (state->cc.s << 7) |   // Sign flag (bit 7)
                      (state->cc.z << 6) |   // Zero flag (bit 6)
                      (state->cc.x5 << 5) |  // X5 flag (bit 5, undocumented)
                      (state->cc.ac << 4) |  // Auxiliary carry (bit 4)
                      (0 << 3) |             // Bit 3 is always 0
                      (state->cc.p << 2) |   // Parity flag (bit 2)
                      (state->cc.v << 1) |   // V overflow flag (bit 1, undocumented)
                      (state->cc.cy);        // Carry flag (bit 0)

        // Step 5: Store the PSW byte at the new stack pointer location
        state->memory[state->sp] = psw;

        states = 13;
    } break;
    case 0xf6: // ORI d8
        state->a = state->a | opcode[1];
        LogicFlagsA(state, 0);
        state->pc++;
        states = 7;
        break;
    case 0xf7: // RST 6
        rst(state, 6, 0);
        states = 12;
        break;
    case 0xf8: // RM
        states = 6;
        if (1 == state->cc.s) {
            returnToCaller(state);
            states = 12;
        }
        break;
    case 0xf9: // SPHL
        state->sp = (state->h << 8) | state->l;
        states = 6;
        break;
    case 0xfa: // JM Addr
        if (1 == state->cc.s) {
            state->pc = ((opcode[2] << 8) | opcode[1]);
            states = 10;
        } else {
            state->pc += 2;
            states = 7;
        }
        break;

    case 0xfb: // EI
        state->int_enable_delay = 2;
        states = 4;
        break;
    case 0xfc: // CM Addr
        if (1 == state->cc.s) {
            call(state, (opcode[2] << 8) | opcode[1]);
            states = 18;
        } else {
            state->pc += 2;
            states = 9;
        }
        break;
    case 0xfd: // JX5 (undocumented): Jump if X5
        if (1 == state->cc.x5) {
            state->pc = ((opcode[2] << 8) | opcode[1]);
            states = 10;
        } else {
            state->pc += 2;
            states = 7;
        }
        break;
    case 0xfe: // CPI d8
    {
        uint8_t x = state->a - opcode[1];
        state->cc.z = (x == 0);
        state->cc.s = (0x80 == (x & 0x80));
        state->cc.p = parity(x, 8);
        state->cc.cy = (state->a < opcode[1]);
        state->cc.ac = (state->a & 0x0f) < (opcode[1] & 0x0f);
        state->pc++;
        states = 7;
    } break;
    case 0xff: // RST 7
        rst(state, 7, 0);
        states = 12;
        break;
    }

    if (!stats->min_sp_set || state->sp < stats->min_sp) {
        stats->min_sp = state->sp;
        stats->min_sp_set = true;
    }

    stats->total_tstates += states;

    if (state->int_enable_delay > 0) {
        state->int_enable_delay--;
        if (state->int_enable_delay == 0) {
            state->int_enable = 1;
        }
    }

    return state->hlt_enable ? 1 : 0;
}
