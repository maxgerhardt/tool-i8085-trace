# Spec — Tri-state pins + logic-net wiring

Date: 2026-07-30
Repo: `tool-i8085-trace`
Builds on: the plugin introspection contract (`snapshot`, `I8085PinBus`) and the
per-pin board view.

## Problem

Peripheral outputs are currently either simple push-pull values or hardcoded
side effects. The MC6850 asserts the 8085's RST 7.5 by directly poking
`state->r7_latch` — a fixed "if UART interrupt, trigger RST 7.5" shortcut. Real
boards wire chip pins through discrete logic: the MC6850 `/IRQ` is an
**open-drain, active-low** output (hi-Z when idle, driving 0 when asserted, no
internal pull-up); a board adds a pull-up and may route it through an inverter
before it reaches an 8085 interrupt pin. We want to model that generically so
arbitrary wire-ups — not just the one baked-in path — can be simulated.

## Goals

- A **tri-state** pin model: every driver pin contributes one of {0, 1, Z}.
  Open-drain outputs (e.g. MC6850 `/IRQ`) drive 0 or release to Z.
- A **core logic-net engine** that resolves nodes from their drivers + pulls,
  evaluates combinational gates, and delivers resolved levels back to chip
  input pins — each CPU step.
- A **text netlist** (`--netlist <file>`) describing nodes, pulls, pin
  connections, and gates.
- The **8085 CPU exposed as a peer** (`cpu.*`) whose interrupt pins are net
  endpoints. With a netlist loaded, the net drives them and the old direct
  `r7_latch` poke is suppressed. With no netlist, behavior is unchanged.

Non-goals (v1): sequential logic (flip-flops, latches, edge detection); analog
timing / propagation delay; buses wider than the existing 32-pin `I8085PinBus`.

## Decisions (confirmed)

- **Netlist ⇒ you wire interrupts yourself.** Loading a netlist makes the net
  authoritative for the CPU interrupt pins; there is no per-pin auto-fallback.
  If you load a netlist and do not wire an interrupt, that interrupt will not
  fire. (No netlist ⇒ today's direct path is untouched.)
- **A floating net is an error worth surfacing.** A node with only Z drivers and
  no pull resolves to X and emits a one-time warning naming the node.

## Design

### 1. Tri-state pin model

`I8085PinBus` already carries a `hi_z` mask; we make it authoritative for
driving. Per step, an output pin contributes:

- `hi_z` bit set  → **Z** (released / not driving).
- else            → **`level` bit** (drives 0 or 1).

There is no separate "open-drain vs push-pull" flag: a chip encodes open-drain
simply by releasing to Z instead of driving 1. The MC6850 `/IRQ` becomes:

- interrupt asserted (`rdrFull && rxIntEnabled`) → `level=0, hi_z=0` (drive low)
- otherwise                                      → `hi_z=1` (release to Z)

The board view already renders `hi_z` pins distinctly (`z-` / dim), so
tri-stating shows immediately.

### 2. Pin naming + the wiring contract

Global pin name: **`kind@base.Pin`**.

- Single-pin buses use the field name: `mc6850@0xDE.IRQ`, `mc6850@0xDE.CTS`.
- Multi-pin buses index by bit: `i8255@0x00.A3` = bus `A`, bit 3; `pit8254@0x40.GATE1`.
- The CPU peer: `cpu.RST7.5`, `cpu.RST6.5`, `cpu.RST5.5`, `cpu.TRAP`, `cpu.INTR`,
  `cpu.SID`, `cpu.SOD`.

The engine **reuses `snapshot()`** to read each chip's output-pin drive states
(the `I8085PinBus` `level`/`hi_z`/`is_input` it already returns — a pure read).
Only one new plugin callback is added, to deliver resolved levels into input
pins:

```c
// Optional (plugin API v5). Deliver a resolved net level to one of this
// plugin's INPUT pins (named as in snapshot's pin buses, bit-indexed for
// multi-pin buses, e.g. "CTS" or "A3"). Called by the net engine each step
// for every wired input pin. Chips apply it to the value their read handlers
// / behaviour observe (e.g. i8255 input latch, mc6850 CTS/DCD).
void (*pin_set)(void *ctx, const char *pin, UINT8 level);
```

Plugin ABI bumps **4 → 5**. `pin_set` is optional (a chip with no wired inputs
never needs it). Chips already expose their pins via `snapshot`, so no separate
pin-declaration API is required.

### 3. Net engine (core, combinational)

New core unit `src/logic_net.{h,cpp}`, driven from the runtime each CPU step
(after instruction execution, before the next fetch):

1. **Read drivers.** For every node, gather contributions from its bound output
   pins (via cached `snapshot`) and gate outputs. A pin with `hi_z=1`
   contributes nothing.
2. **Resolve each node:**
   - ≥1 active driver, all agree → that level.
   - active drivers conflict (0 and 1) → **X**, warn once per node.
   - no active drivers → pull decides: `up`→1, `down`→0, none→**X** + one-time
     "floating net" warning.
   Wired-AND emerges naturally: an open-drain 0 beats a pull-up (→0); released
   (Z) leaves the pull-up (→1).
3. **Evaluate gates** from their (now-resolved) input nodes, driving their
   output nodes.
4. **Settle:** repeat (2)-(3) until nodes stop changing or a cap (~16
   iterations) is hit; a non-settling net holds its last state and warns
   (oscillation).
5. **Deliver inputs.** For every bound input pin, call the owning chip's
   `pin_set(pin, level)` (and set `cpu.*` latches directly for CPU pins).

Gate types (v1): `INV`, `BUF`, `AND`, `OR`, `NAND`, `NOR` (multi-input for the
2+-input gates), push-pull outputs. An X input propagates as X.

### 4. Netlist file (`--netlist <file>`)

One directive per line; `#` comments; blank lines ignored.

```
pull  <node>  up | down            # optional pull resistor on a node
wire  <node>  <pin>                # bind a chip/cpu pin to a node (repeatable)
gate  <type>  <name>  <in...> <out># instantiate a gate; last token is the output node
```

Example — MC6850 `/IRQ` → pull-up → inverter → RST 7.5:

```
pull  IRQ_L   up
wire  IRQ_L   mc6850@0xDE.IRQ
gate  inv  U1  IRQ_L  RST_IN
wire  RST_IN  cpu.RST7.5
```

Nodes are created on first mention. Pin bindings are resolved after all plugins
have loaded, matching `kind@base` against each plugin's `snapshot` info and the
pin name against its buses; an unresolved pin or unknown gate type is a fatal
load error naming the offending line.

### 5. CPU peer + interrupt integration

The core presents a built-in `cpu` peer (not a loadable plugin) exposing pins
`RST7.5/6.5/5.5`, `TRAP`, `INTR`, `SID` (inputs to the CPU) and `SOD` (output).
It participates in the net like any chip: input pins receive resolved levels;
`SOD` drives from the CPU's serial-out latch.

A host flag **`wiring_active`** (set once a netlist loads) is exposed to plugins.
When set:

- The core drives the CPU interrupt latches (`r7_latch`, etc.) from the resolved
  `cpu.*` nodes each step.
- The MC6850 **stops** poking `r7_latch` directly; it only drives its `/IRQ`
  pin. (Guarded by `wiring_active`.)

When unset (no netlist) every chip behaves exactly as today.

### 6. Board view (synthetic peers, no boardview change)

The `cpu` and the net are surfaced through the **existing peer-introspection
path**: the runtime's `peer_count`/`peer_snapshot` append two synthetic peers
after the loaded plugins — `cpu` (its interrupt/serial pins) and `net` (one
field per node, value `0`/`1`/`float`/`X`). Because `boardview` already renders
whatever peers report, both appear with **zero boardview changes**:

```
[net] IRQ_L=0 RST_IN=1
[cpu] RST7.5=1 RST6.5=0 ...
```

Combined with the existing per-pin rows (which already show `z` for tri-stated
source pins), this lets you watch an edge propagate `IRQ_L: 1→0` through `U1`
into `RST7.5`. Ordering the synthetic peers last for grouping is cosmetic and
deferred.

## Components / boundaries

- `src/logic_net.{h,cpp}` — netlist parse, node/gate model, per-step resolve +
  settle, driver read via `snapshot`, input delivery via `pin_set`. Self-
  contained; depends only on the plugin/host API and the runtime's peer list.
- `io_runtime.cpp` — owns the `logic_net` instance; ticks it each step after
  plugin `on_step`; exposes `wiring_active` on the host API; provides the `cpu`
  and `net` synthetic peers and includes them in `peer_count`/`peer_snapshot`.
- `include/i8085_io_plugin.h` — add `pin_set` (plugin ABI v5), `wiring_active`
  on `I8085HostAPI` (host API v3).
- `plugins/mc6850.cpp` — open-drain `/IRQ` pin; suppress direct `r7_latch` when
  `wiring_active`; accept `CTS`/`DCD` via `pin_set`.
- `plugins/i8255.cpp` — accept input-pin levels via `pin_set` (supersedes the
  static `pa`/`pb`/`pc` config when a pin is wired).
- `plugins/boardview.cpp` — unchanged: the `cpu`/`net` synthetic peers render
  through the existing peer path.

## Testing strategy

1. **Net resolution units** (headless harness): wired-AND (OD-low + pull-up),
   driver conflict → X + warning, floating (all-Z, no pull) → X + warning, each
   gate's truth table, and settle over a 2-gate chain.
2. **End-to-end interrupt** — netlist `IRQ_L(pull-up) → inv U1 → cpu.RST7.5`.
   A program enables MC6850 RX interrupt and EI; feed an RX byte; assert:
   (a) `mc6850@0xDE.IRQ` drives 0 on arrival, (b) `IRQ_L` resolves 0,
   (c) `RST_IN`/`cpu.RST7.5` resolves 1, (d) the CPU vectors to the RST 7.5 ISR
   (e.g. the ISR writes a sentinel we observe).
3. **No-netlist regression** — without `--netlist`, the MC6850 RX interrupt
   still drives RST 7.5 the old way; existing PIT/8255/memory tests unaffected.
4. **Netlist errors** — unknown pin, unknown gate type, and a floating net each
   produce the specified fatal error / warning.

## Rollout / compatibility

- Plugin ABI 4→5, host API 2→3 — additive; all first-party plugins rebuild
  together (CI already globs every plugin). No netlist ⇒ no behavior change.
- The `--netlist` flag is optional and off by default.
