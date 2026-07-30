#pragma once

#include "i8085_cpu.h"
#include <stddef.h>

#ifdef __cplusplus
extern "C" {
#endif

#define I8085_IO_PLUGIN_ABI_VERSION 5u
// The single entry point every plugin must export (see I8085IoPluginInitFn).
#define I8085_IO_PLUGIN_INIT_SYMBOL "i8085_io_plugin_init"
#define I8085_HOST_API_VERSION 3u

// --- Introspection / self-description ---------------------------------------
// A plugin may optionally describe its live state (see snapshot, below) as a
// flat list of named fields. This lets a generic "board view" consumer render
// any peripheral -- including internal state that never appears on the bus,
// e.g. a free-running timer's counter -- without hardcoding chip knowledge.
enum {
    I8085_FIELD_U8 = 0,   // small unsigned, print decimal
    I8085_FIELD_U16 = 1,  // unsigned, print decimal
    I8085_FIELD_U32 = 2,  // unsigned, print decimal
    I8085_FIELD_HEX = 3,  // unsigned, print 0x-hex
    I8085_FIELD_BOOL = 4, // 0/1
    I8085_FIELD_STR = 5,  // use `s` instead of `u`
    I8085_FIELD_PINS = 6  // a bit-bus of individually-directed pins (use `bus`)
};

// A bus of up to 32 individually-directed pins. Lets a chip expose e.g. an
// 8255 port whose bits may be a mix of inputs and outputs (Port C splits into
// independently-directed nibbles), so a generic viewer can draw each pin with
// its own direction and level -- inward arrow + level for inputs, driven
// (glowing) level for outputs -- without any chip-specific knowledge.
typedef struct I8085PinBus {
    UINT8 width;     // number of pins in use, bit 0 = pin 0
    UINT32 level;    // bit i = current logic level of pin i (0/1)
    UINT32 is_input; // bit i = 1 -> pin i is an input (else an output)
    UINT32 hi_z;     // bit i = 1 -> pin i is tri-stated / not driven (optional)
} I8085PinBus;

typedef struct I8085StateField {
    const char *name;         // static or ctx-lived string (must outlive the call)
    UINT8 kind;               // one of I8085_FIELD_*
    UINT32 u;                 // numeric value (kinds U8/U16/U32/HEX/BOOL)
    const char *s;            // string value (kind STR); else NULL
    const I8085PinBus *bus;   // pin bus (kind PINS); else NULL. Must outlive call
} I8085StateField;

// What a plugin instance is and where it lives on the bus.
typedef struct I8085PluginInfo {
    const char *kind; // e.g. "mc6850", "pit8254", "memory"; NULL = opaque
    UINT16 base;      // first I/O port owned (0 if not port-mapped)
    UINT16 span;      // number of consecutive ports owned
} I8085PluginInfo;

// Host services handed to a plugin at init. The core owns all the platform
// plumbing (sockets, terminal spawning, stdout framing) behind a simple byte-
// channel abstraction, so a plugin that needs a UART backend does not carry any
// #ifdef _WIN32 socket code -- it just opens channels and pumps bytes.
typedef struct I8085HostAPI {
    UINT32 abi_version;
    // Open a byte channel from a spec string:
    //   "tcp=PORT"       listen on localhost:PORT, bridge one client
    //   "window[=PORT]"  like tcp (PORT optional -> auto-assign) and also spawn
    //                    an i8085-console terminal window bound to it
    //   "gdbtx"          write-only, GDB/MI-safe stdout mirror (CR-free, LF-framed)
    //   "stdout"         write-only raw stdout mirror
    // Returns an opaque channel handle, or NULL on error (errbuf filled).
    void *(*channel_open)(const char *spec, char *errbuf, size_t errbuf_len);
    // Nonblocking: copy up to maxlen received bytes into buf; returns the count
    // (0 if none currently available). Write-only channels always return 0.
    int (*channel_read)(void *chan, void *buf, int maxlen);
    // Queue bytes to transmit; returns the number accepted.
    int (*channel_write)(void *chan, const void *buf, int len);
    void (*channel_close)(void *chan);

    // --- Peer introspection (host API v2) ---
    // Enumerate the other loaded plugins so a viewer can render the whole board.
    // peer_count() is the number of loaded plugin instances (including self).
    int (*peer_count)(void);
    // Snapshot peer `idx` in [0,peer_count): fills out_info and up to max_fields
    // of out_fields from that peer's snapshot callback. Returns the number of
    // fields written (>=0), or -1 if idx is out of range. A peer that does not
    // implement snapshot yields info.kind=NULL and 0 fields. This is a pure
    // debug read: it never disturbs peer state (see snapshot, above).
    int (*peer_snapshot)(int idx, I8085PluginInfo *out_info, I8085StateField *out_fields, int max_fields);

    // 1 once a --netlist is loaded: the logic net is authoritative for wired
    // pins (incl. the CPU interrupt lines). Chips that had a hardcoded effect
    // (e.g. MC6850 poking RST 7.5) must defer to the net when this is set.
    UINT32 wiring_active;
} I8085HostAPI;

// The API struct is append-only: new callbacks are only ever added at the END,
// and the host zero-initializes the struct before calling a plugin's init and
// null-checks every callback before use. A future ABI revision bumps
// I8085_IO_PLUGIN_ABI_VERSION; the host requires plugins to report exactly the
// current version.
typedef struct I8085IoPluginAPI {
    UINT32 abi_version;
    void (*destroy)(void *ctx);
    void (*on_reset)(void *ctx, State8085 *state);
    void (*on_step)(void *ctx, State8085 *state, UINT64 step, UINT64 tstates);
    void (*on_io_write)(void *ctx, State8085 *state, UINT8 port, UINT8 value);
    void (*on_io_pre_read)(void *ctx, State8085 *state, UINT8 port);
    void (*on_io_post_read)(void *ctx, State8085 *state, UINT8 port, UINT8 value);

    // Called for every CPU memory write, BEFORE the byte is stored.
    // Returns the value to actually store: return `val` to allow the write, or
    // return the current state->memory[addr] to veto it (write-protection).
    // When several plugins are loaded the value is chained through each in load
    // order. Plugins that do not model memory leave this NULL.
    UINT8 (*on_mem_write)(void *ctx, State8085 *state, UINT16 addr, UINT8 val);

    // Optional (host/plugin API v3). Describe this instance's identity and live
    // state for a board-view consumer. Fill out_info and up to max_fields of
    // out_fields; return the number of fields written. Called on demand (never
    // on the hot path). Field name/string pointers must outlive the call.
    //
    // MUST be side-effect-free: a pure "debug read". Unlike a CPU bus read
    // (on_io_post_read), snapshot is invoked out of band by observers, so it
    // MUST NOT do anything a real read would -- do not clear interrupts, consume
    // a received byte, advance a FIFO/read-latch, or mutate any register. Read
    // internal state directly; never route through the on_io_* handlers.
    int (*snapshot)(void *ctx, I8085PluginInfo *out_info, I8085StateField *out_fields, int max_fields);

    // Optional (plugin API v5). Deliver a resolved logic-net level to one of
    // this plugin's INPUT pins, named as in snapshot's pin buses (bit-indexed
    // for multi-pin buses, e.g. "CTS" or "A3"). level is 0, 1, or 2 (X/unknown).
    void (*pin_set)(void *ctx, const char *pin, UINT8 level);
} I8085IoPluginAPI;

// Every plugin exports i8085_io_plugin_init with this signature. `host` is the
// host byte-channel table (never NULL); a plugin that needs no host services
// simply ignores it.
typedef int (*I8085IoPluginInitFn)(const char *config, const I8085HostAPI *host, void **out_ctx,
                                   I8085IoPluginAPI *out_api, char *errbuf, size_t errbuf_len);

#ifdef __cplusplus
}
#endif
