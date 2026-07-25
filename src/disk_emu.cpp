//----------------------------------------------------------------------------
// Disk Emulator for I8085 Trace
//
// Maps I/O port commands to host file operations. Files live in a
// specified directory (--disk=./disk/). Supports one read file and
// one write file open simultaneously.
//----------------------------------------------------------------------------

#include "disk_emu.h"
#include <cinttypes>
#include <cstdio>
#include <cstring>
#include <string>

static struct {
    std::string dir;
    FILE *readFp;
    FILE *writeFp;
    std::string readName;
    std::string writeName;
    UINT8 addrLo;
    UINT8 addrHi;
    UINT8 dataByte;
    UINT8 status;
    bool trace;
    bool initialized;
    UINT64 bytesRead;
    UINT64 bytesWritten;
} gDisk;

int disk_emu_init(const char *dir, int trace) {
    gDisk.dir = dir ? dir : ".";
    // Ensure trailing slash
    if (!gDisk.dir.empty() && gDisk.dir.back() != '/')
        gDisk.dir += '/';
    gDisk.readFp = nullptr;
    gDisk.writeFp = nullptr;
    gDisk.addrLo = 0;
    gDisk.addrHi = 0;
    gDisk.dataByte = 0;
    gDisk.status = DISK_STATUS_OK;
    gDisk.trace = (trace != 0);
    gDisk.initialized = true;
    gDisk.bytesRead = 0;
    gDisk.bytesWritten = 0;
    if (gDisk.trace)
        fprintf(stderr, "[DISK] initialized, dir=%s\n", gDisk.dir.c_str());
    return 0;
}

void disk_emu_destroy(void) {
    if (gDisk.readFp) {
        fclose(gDisk.readFp);
        if (gDisk.trace)
            fprintf(stderr, "[DISK] closed read file '%s' (%" PRIu64 " bytes read)\n",
                    gDisk.readName.c_str(), gDisk.bytesRead);
    }
    if (gDisk.writeFp) {
        fclose(gDisk.writeFp);
        if (gDisk.trace)
            fprintf(stderr, "[DISK] closed write file '%s' (%" PRIu64 " bytes written)\n",
                    gDisk.writeName.c_str(), gDisk.bytesWritten);
    }
    gDisk.readFp = nullptr;
    gDisk.writeFp = nullptr;
    gDisk.initialized = false;
}

int disk_emu_active(void) {
    return gDisk.initialized ? 1 : 0;
}

// Extract filename from 8085 memory at addr, length len
static std::string extractFilename(State8085 *state, UINT16 addr, UINT8 len) {
    std::string name;
    for (int i = 0; i < len && i < 255; i++) {
        char c = (char)state->memory[(addr + i) & 0xFFFF];
        if (c == '\0') break;
        name += c;
    }
    return name;
}

static void diskCommand(State8085 *state, UINT8 cmd) {
    UINT16 addr = (UINT16)gDisk.addrHi << 8 | gDisk.addrLo;

    switch (cmd) {
    case DISK_CMD_OPEN_READ: {
        if (gDisk.readFp) {
            fclose(gDisk.readFp);
            gDisk.readFp = nullptr;
        }
        UINT8 len = gDisk.dataByte;
        std::string name = extractFilename(state, addr, len);
        std::string path = gDisk.dir + name;
        gDisk.readFp = fopen(path.c_str(), "rb");
        if (gDisk.readFp) {
            gDisk.status = DISK_STATUS_OK;
            gDisk.readName = name;
            gDisk.bytesRead = 0;
            if (gDisk.trace)
                fprintf(stderr, "[DISK] OPEN_READ '%s' -> ok\n", path.c_str());
        } else {
            gDisk.status = DISK_STATUS_ERROR;
            if (gDisk.trace)
                fprintf(stderr, "[DISK] OPEN_READ '%s' -> FAILED\n", path.c_str());
        }
        break;
    }
    case DISK_CMD_OPEN_WRITE: {
        if (gDisk.writeFp) {
            fclose(gDisk.writeFp);
            gDisk.writeFp = nullptr;
        }
        UINT8 len = gDisk.dataByte;
        std::string name = extractFilename(state, addr, len);
        std::string path = gDisk.dir + name;
        gDisk.writeFp = fopen(path.c_str(), "wb");
        if (gDisk.writeFp) {
            gDisk.status = DISK_STATUS_OK;
            gDisk.writeName = name;
            gDisk.bytesWritten = 0;
            if (gDisk.trace)
                fprintf(stderr, "[DISK] OPEN_WRITE '%s' -> ok\n", path.c_str());
        } else {
            gDisk.status = DISK_STATUS_ERROR;
            if (gDisk.trace)
                fprintf(stderr, "[DISK] OPEN_WRITE '%s' -> FAILED\n", path.c_str());
        }
        break;
    }
    case DISK_CMD_READ_BYTE: {
        if (!gDisk.readFp) {
            gDisk.status = DISK_STATUS_ERROR;
            gDisk.dataByte = 0;
            break;
        }
        int ch = fgetc(gDisk.readFp);
        if (ch == EOF) {
            gDisk.status = DISK_STATUS_EOF;
            gDisk.dataByte = 0;
        } else {
            gDisk.status = DISK_STATUS_OK;
            gDisk.dataByte = (UINT8)ch;
            gDisk.bytesRead++;
        }
        break;
    }
    case DISK_CMD_WRITE_BYTE: {
        if (!gDisk.writeFp) {
            gDisk.status = DISK_STATUS_ERROR;
            break;
        }
        if (fputc(gDisk.dataByte, gDisk.writeFp) == EOF) {
            gDisk.status = DISK_STATUS_ERROR;
        } else {
            gDisk.status = DISK_STATUS_OK;
            gDisk.bytesWritten++;
        }
        break;
    }
    case DISK_CMD_CLOSE: {
        if (gDisk.readFp) {
            fclose(gDisk.readFp);
            if (gDisk.trace)
                fprintf(stderr, "[DISK] CLOSE read '%s' (%" PRIu64 " bytes)\n",
                        gDisk.readName.c_str(), gDisk.bytesRead);
            gDisk.readFp = nullptr;
        }
        if (gDisk.writeFp) {
            fclose(gDisk.writeFp);
            if (gDisk.trace)
                fprintf(stderr, "[DISK] CLOSE write '%s' (%" PRIu64 " bytes)\n",
                        gDisk.writeName.c_str(), gDisk.bytesWritten);
            gDisk.writeFp = nullptr;
        }
        gDisk.status = DISK_STATUS_OK;
        break;
    }
    case DISK_CMD_REWIND: {
        if (gDisk.readFp) {
            fseek(gDisk.readFp, 0, SEEK_SET);
            gDisk.status = DISK_STATUS_OK;
            gDisk.bytesRead = 0;
            if (gDisk.trace)
                fprintf(stderr, "[DISK] REWIND '%s'\n", gDisk.readName.c_str());
        } else {
            gDisk.status = DISK_STATUS_ERROR;
        }
        break;
    }
    default:
        gDisk.status = DISK_STATUS_ERROR;
        if (gDisk.trace)
            fprintf(stderr, "[DISK] unknown command 0x%02X\n", cmd);
        break;
    }
}

void disk_emu_on_io_write(State8085 *state, UINT8 port, UINT8 value) {
    if (!gDisk.initialized) return;

    switch (port) {
    case DISK_PORT_CMD:
        diskCommand(state, value);
        break;
    case DISK_PORT_DATA:
        gDisk.dataByte = value;
        break;
    case DISK_PORT_ADRL:
        gDisk.addrLo = value;
        break;
    case DISK_PORT_ADRH:
        gDisk.addrHi = value;
        break;
    }
}

void disk_emu_on_io_pre_read(State8085 *state, UINT8 port) {
    if (!gDisk.initialized) return;

    switch (port) {
    case DISK_PORT_CMD:
        // Status read
        state->io[DISK_PORT_CMD] = gDisk.status;
        break;
    case DISK_PORT_DATA:
        // Data read (result of READ_BYTE)
        state->io[DISK_PORT_DATA] = gDisk.dataByte;
        break;
    }
}
