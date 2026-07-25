#pragma once

#include "i8085_cpu.h"

#ifdef __cplusplus
extern "C" {
#endif

// Disk emulator I/O port protocol:
//   OUT 0xF0 = command    IN 0xF0 = status (0=ok, 1=EOF, 0xFF=error)
//   OUT 0xF1 = data       IN 0xF1 = data (read byte result)
//   OUT 0xF2 = addr lo    OUT 0xF3 = addr hi
//
// Commands (OUT 0xF0):
//   0x01  OPEN_READ   filename at memory[addr], len via prior OUT 0xF1
//   0x02  OPEN_WRITE  filename at memory[addr], len via prior OUT 0xF1
//   0x03  READ_BYTE   result via IN 0xF1, status via IN 0xF0
//   0x04  WRITE_BYTE  byte from prior OUT 0xF1
//   0x05  CLOSE       close all open files
//   0x06  REWIND      seek read file to start

#define DISK_PORT_CMD   0xF0
#define DISK_PORT_DATA  0xF1
#define DISK_PORT_ADRL  0xF2
#define DISK_PORT_ADRH  0xF3

#define DISK_CMD_OPEN_READ  0x01
#define DISK_CMD_OPEN_WRITE 0x02
#define DISK_CMD_READ_BYTE  0x03
#define DISK_CMD_WRITE_BYTE 0x04
#define DISK_CMD_CLOSE      0x05
#define DISK_CMD_REWIND     0x06

#define DISK_STATUS_OK    0x00
#define DISK_STATUS_EOF   0x01
#define DISK_STATUS_ERROR 0xFF

int  disk_emu_init(const char *dir, int trace);
void disk_emu_destroy(void);
void disk_emu_on_io_write(State8085 *state, UINT8 port, UINT8 value);
void disk_emu_on_io_pre_read(State8085 *state, UINT8 port);
int  disk_emu_active(void);

#ifdef __cplusplus
}
#endif
