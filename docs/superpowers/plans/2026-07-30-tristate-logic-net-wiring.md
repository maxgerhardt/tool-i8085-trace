# Tri-state Pins + Logic-Net Wiring Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Simulate arbitrary board wiring — tri-state (open-drain) chip pins routed through pull resistors and combinational gates into other chip pins (including the 8085's interrupt pins) — via a declarative netlist, replacing the hardcoded MC6850-IRQ→RST7.5 shortcut.

**Architecture:** A pure combinational logic-net core (`src/logic_net.{h,cpp}`) parses a netlist and, each CPU step, reads driver states, resolves every node (wired logic + pull), settles gates to a fixpoint, and delivers resolved levels back to input pins — all through an abstract `Host` callback interface. `io_runtime` implements that `Host` over the loaded plugins (reading drivers from the existing `snapshot()` pin buses, delivering inputs via a new `pin_set()` callback) plus a built-in `cpu` peer. With a netlist loaded, the net drives the CPU interrupt latches and the MC6850 stops poking them directly.

**Tech Stack:** C++17, CMake + Ninja, MinGW-w64 (local: `C:/Users/Max/Downloads/mingw64/bin`). No unit-test framework in the repo — unit tests are standalone `assert`-based executables; integration tests run `i8085-trace` on crafted programs and inspect output.

## Global Constraints

- **Toolchain (local build):** `export PATH="/c/Users/Max/Downloads/mingw64/bin:$PATH"`, then `cmake -S . -B build -G Ninja -DCMAKE_BUILD_TYPE=Release && cmake --build build`.
- **Plugin ABI:** bump `I8085_IO_PLUGIN_ABI_VERSION` `4u → 5u`; **host API** `I8085_HOST_API_VERSION` `2u → 3u`. Additive only (append struct fields at END). The runtime requires an exact plugin ABI match, so all first-party plugins rebuild together.
- **Backward compatibility:** with **no** `--netlist`, behavior is byte-for-byte unchanged (the MC6850 keeps poking `r7_latch`; no net ticks).
- **`snapshot()` is a pure debug read** — never mutate state in it. Driver reads for the net go through `snapshot()` and must stay side-effect-free.
- **Commit trailer (every commit):**
  `Co-Authored-By: Claude Opus 4.8 <noreply@anthropic.com>`
- **Repo:** `tool-i8085-trace` is the live source. Do not touch the stale `i8085-trace/` folder.
- **CI/push (only when a task says to):** `git push "https://x-access-token:$(gh auth token)@github.com/maxgerhardt/tool-i8085-trace.git" main`.

---

### Task 1: logic_net skeleton + netlist parser

**Files:**
- Create: `src/logic_net.h`
- Create: `src/logic_net.cpp`
- Create: `test/logic_net_test.cpp`
- Modify: `CMakeLists.txt` (add a `test_logic_net` executable)

**Interfaces:**
- Produces (consumed by all later logic_net tasks and the runtime):
  ```cpp
  namespace ln {
  enum Drive { DRV_0 = 0, DRV_1 = 1, DRV_Z = 2 };
  enum Level { LVL_0 = 0, LVL_1 = 1, LVL_X = 2 };
  enum Pull  { PULL_NONE = 0, PULL_UP = 1, PULL_DOWN = 2 };
  enum GateType { G_INV, G_BUF, G_AND, G_OR, G_NAND, G_NOR };

  struct Host {                                   // supplied by the runtime / tests
      void *ctx;
      Drive (*read_output)(void *ctx, int handle);        // driver state of an output pin
      void  (*write_input)(void *ctx, int handle, int level); // deliver 0/1/2(X) to an input pin
      void  (*warn)(void *ctx, const char *msg);
  };

  class Net {
  public:
      bool parse(const std::string &text, std::string &err); // Task 1
      bool bind(int (*resolve)(void *ctx, const char *pin, int *is_output),
                void *ctx, std::string &err);                // Task 4
      void step(const Host &host);                           // Tasks 2-3
      int node_count() const;                                // Task 4
      const std::string &node_name(int i) const;             // Task 4
      Level node_level(int i) const;                         // Task 4
  };
  } // namespace ln
  ```
- Parser model (private, but later tasks rely on these being populated): a node has a `name` and `Pull pull`; an endpoint has `{int node; std::string pin; int handle=-1; bool is_output=false;}`; a gate has `{GateType type; std::vector<int> in_nodes; int out_node;}`. Nodes are created on first mention (case-sensitive names). Directives: `pull <node> up|down`, `wire <node> <pin>`, `gate <type> <name> <in...> <out>` (last token = output node). `#` starts a comment; blank lines ignored. Unknown directive/gate-type ⇒ `parse` returns false with `err` = `line N: <reason>`.

- [ ] **Step 1: Write the failing test**

Create `test/logic_net_test.cpp`:
```cpp
#include "../src/logic_net.h"
#include <cassert>
#include <cstdio>
#include <string>

static void test_parse_basic() {
    ln::Net net;
    std::string err;
    const char *nl =
        "# comment\n"
        "pull  IRQ_L  up\n"
        "wire  IRQ_L  mc6850@0xDE.IRQ\n"
        "gate  inv  U1  IRQ_L  RST_IN\n"
        "wire  RST_IN  cpu.RST7.5\n";
    bool ok = net.parse(nl, err);
    assert(ok && err.empty());
    // IRQ_L, RST_IN => 2 nodes.
    assert(net.node_count() == 2);
}

static void test_parse_error() {
    ln::Net net;
    std::string err;
    bool ok = net.parse("gate xor X1 A B C\n", err);
    assert(!ok);
    assert(err.find("line 1") != std::string::npos);
}

int main() {
    test_parse_basic();
    test_parse_error();
    printf("logic_net_test: PASS\n");
    return 0;
}
```

- [ ] **Step 2: Create `src/logic_net.h`** (the `Interfaces` block above, verbatim, wrapped in `#pragma once` and `#include <string>`, `#include <vector>`). Give `Net` private members: `std::vector<Node> nodes_; std::vector<Endpoint> endpoints_; std::vector<Gate> gates_;` and a private `int nodeIndex(const std::string&)` that finds-or-creates.

- [ ] **Step 3: Create `src/logic_net.cpp`** implementing only `parse`, `node_count`, `node_name`, `node_level` (the last two/`step`/`bind` may be stubs returning defaults for now — a stub `step`/`bind` is fine this task). Tokenize each line on whitespace; dispatch on the first token; map gate-type strings (`inv,buf,and,or,nand,nor`) to `GateType` (unknown ⇒ error). For `gate`, the tokens after `<name>` are input node names except the last which is the output node.

- [ ] **Step 4: Add the CMake test target.** In `CMakeLists.txt`, after the plugins, append:
```cmake
# Standalone unit test for the logic-net core (assert-based).
add_executable(test_logic_net test/logic_net_test.cpp src/logic_net.cpp)
target_include_directories(test_logic_net PRIVATE src)
```

- [ ] **Step 5: Build and run — verify it passes**

Run:
```bash
export PATH="/c/Users/Max/Downloads/mingw64/bin:$PATH"
cmake -S . -B build -G Ninja -DCMAKE_BUILD_TYPE=Release >/dev/null && cmake --build build --target test_logic_net
./build/test_logic_net.exe
```
Expected: `logic_net_test: PASS`. (First run the test BEFORE writing `logic_net.cpp` bodies to see it fail to build/link — that is the RED step.)

- [ ] **Step 6: Commit**
```bash
git add src/logic_net.h src/logic_net.cpp test/logic_net_test.cpp CMakeLists.txt
git commit -m "feat: logic_net netlist parser + unit-test harness

Co-Authored-By: Claude Opus 4.8 <noreply@anthropic.com>"
```

---

### Task 2: Node resolution (drivers + pull + conflict/floating)

**Files:**
- Modify: `src/logic_net.cpp` (implement `step` resolution; no gates yet)
- Modify: `test/logic_net_test.cpp` (add resolution tests)

**Interfaces:**
- Consumes: `ln::Net`, `ln::Host`, `Drive`/`Level`/`Pull` from Task 1.
- Produces: a working `void Net::step(const Host&)` for **gateless** nets, and node level accessors. Resolution rule for a node: read every bound **output** endpoint's `Drive` via `host.read_output(handle)`. Collect `has0`/`has1` (ignore `DRV_Z`). Then: `has0 && has1` ⇒ `LVL_X` + one-time conflict warning; `has0` ⇒ `LVL_0`; `has1` ⇒ `LVL_1`; neither ⇒ pull decides (`PULL_UP`→`LVL_1`, `PULL_DOWN`→`LVL_0`, `PULL_NONE`→`LVL_X` + one-time floating warning). Warn-once is tracked per node index in a `std::vector<bool>`.

- [ ] **Step 1: Write the failing tests** — append to `test/logic_net_test.cpp` and call them from `main`:
```cpp
// A tiny fake Host backed by arrays, indexed by handle.
struct Fake {
    ln::Drive drv[8];       // per output-endpoint handle
    int in[8];              // last level delivered per input handle
    int warns = 0;
};
static ln::Drive fake_read(void *c, int h) { return ((Fake *)c)->drv[h]; }
static void fake_write(void *c, int h, int lv) { ((Fake *)c)->in[h] = lv; }
static void fake_warn(void *c, const char *) { ((Fake *)c)->warns++; }

// Bind helper: assign handles in wire order via a resolver that hands out an
// incrementing index and marks the pin an output iff its name starts with "O".
static int seq_resolve(void *c, const char *pin, int *is_output) {
    int *n = (int *)c;
    *is_output = (pin[0] == 'O');
    return (*n)++;
}

static void test_wired_and_pullup() {
    ln::Net net; std::string err;
    // Two open-drain outputs O0,O1 on node N with a pull-up; one input I on N.
    net.parse("pull N up\nwire N O0\nwire N O1\nwire N I\n", err);
    int next = 0; net.bind(seq_resolve, &next, err);     // O0=h0, O1=h1, I=h2
    Fake f; ln::Host host{&f, fake_read, fake_write, fake_warn};
    // both released (Z) -> pull-up -> 1
    f.drv[0] = ln::DRV_Z; f.drv[1] = ln::DRV_Z; net.step(host);
    assert(f.in[2] == ln::LVL_1);
    // one drives 0 -> wired-AND -> 0
    f.drv[0] = ln::DRV_0; net.step(host);
    assert(f.in[2] == ln::LVL_0);
}

static void test_conflict_and_float() {
    ln::Net net; std::string err;
    net.parse("wire N O0\nwire N O1\nwire N I\n", err);   // no pull
    int next = 0; net.bind(seq_resolve, &next, err);
    Fake f; ln::Host host{&f, fake_read, fake_write, fake_warn};
    f.drv[0] = ln::DRV_0; f.drv[1] = ln::DRV_1; net.step(host);   // conflict
    assert(f.in[2] == ln::LVL_X && f.warns >= 1);
    f.drv[0] = ln::DRV_Z; f.drv[1] = ln::DRV_Z; net.step(host);   // floating
    assert(f.in[2] == ln::LVL_X);
}
```
> NOTE: `bind` may still be a stub after Task 1. If so, implement the minimal `bind` here (walk `endpoints_`, call `resolve(ctx, pin, &is_output)`, store `handle`/`is_output`; return false + err on handle `< 0`). Task 4 hardens it.

- [ ] **Step 2: Run tests to verify they fail**
Run: `cmake --build build --target test_logic_net && ./build/test_logic_net.exe`
Expected: FAIL (assert abort) — `step` does not yet deliver resolved levels.

- [ ] **Step 3: Implement resolution in `Net::step`.** For each node compute `Level` per the rule in Interfaces; store into a `std::vector<Level> level_` sized to `nodes_`. Then for each **input** endpoint (`!is_output`) call `host.write_input(handle, level_[node])`. Keep a `std::vector<bool> warnedConflict_, warnedFloat_` (resized in `parse`/`bind`) to warn once.

- [ ] **Step 4: Run tests to verify they pass**
Run: `cmake --build build --target test_logic_net && ./build/test_logic_net.exe`
Expected: `logic_net_test: PASS`

- [ ] **Step 5: Commit**
```bash
git add src/logic_net.cpp test/logic_net_test.cpp
git commit -m "feat: logic_net node resolution (wired logic, pull, conflict/float)

Co-Authored-By: Claude Opus 4.8 <noreply@anthropic.com>"
```

---

### Task 3: Gates + settle loop

**Files:**
- Modify: `src/logic_net.cpp` (gate evaluation + iterative settle)
- Modify: `test/logic_net_test.cpp`

**Interfaces:**
- Consumes: resolution from Task 2.
- Produces: `step` now settles combinational gate chains. A gate is a **push-pull driver** on its output node whose level is stored in `std::vector<Level> gateOut_` (sized to `gates_`). Node resolution additionally treats each gate feeding a node as a driver: `gateOut_ == LVL_0` ⇒ contributes `has0`; `LVL_1` ⇒ `has1`; `LVL_X` ⇒ forces node `LVL_X`. Settle: initialize `gateOut_` to `LVL_X`; loop up to 16 times: (a) resolve all nodes, (b) recompute each gate from its input nodes; stop when neither `level_` nor `gateOut_` changed in a pass. If still changing after 16 passes, keep last values and warn once ("oscillation"). Gate truth (X-aware): `INV`(a): 0→1,1→0,X→X. `BUF`(a): a. `AND`: any 0→0; else any X→X; else 1. `OR`: any 1→1; else any X→X; else 0. `NAND`=logical-not of AND (any 0→1; else any X→X; else 0). `NOR`=not of OR (any 1→0; else any X→X; else 1).

- [ ] **Step 1: Write the failing tests** — append and call from `main`:
```cpp
static void test_inverter_chain() {
    ln::Net net; std::string err;
    // O drives IN; inverter U1: IN -> MID; inverter U2: MID -> OUTN; input I on OUTN.
    net.parse("wire IN O\ngate inv U1 IN MID\ngate inv U2 MID OUTN\nwire OUTN I\n", err);
    int next = 0; net.bind(seq_resolve, &next, err);   // O=h0(out), I=h1(in)
    Fake f; ln::Host host{&f, fake_read, fake_write, fake_warn};
    f.drv[0] = ln::DRV_0; net.step(host); assert(f.in[1] == ln::LVL_0); // 0 -> ~0=1 -> ~1=0
    f.drv[0] = ln::DRV_1; net.step(host); assert(f.in[1] == ln::LVL_1);
}

static void test_and_gate() {
    ln::Net net; std::string err;
    net.parse("wire A O0\nwire B O1\ngate and G A B Y\nwire Y I\n", err);
    int next = 0; net.bind(seq_resolve, &next, err);   // O0=h0,O1=h1(out), I=h2(in)
    Fake f; ln::Host host{&f, fake_read, fake_write, fake_warn};
    f.drv[0]=ln::DRV_1; f.drv[1]=ln::DRV_1; net.step(host); assert(f.in[2]==ln::LVL_1);
    f.drv[1]=ln::DRV_0; net.step(host);                    assert(f.in[2]==ln::LVL_0);
}
```
> The single-char `seq_resolve` marks a pin an output iff its name starts with `O`; `A`/`B` are node names here, and their bound endpoints are `O0`/`O1`, so handles line up as annotated.

- [ ] **Step 2: Run to verify fail**
Run: `cmake --build build --target test_logic_net && ./build/test_logic_net.exe`
Expected: FAIL — gates not evaluated (inputs get X or stale).

- [ ] **Step 3: Implement gates + settle** in `Net::step` per Interfaces. Add a private `Level evalGate(const Gate&, const std::vector<Level>&)`. Precompute, for each node, the list of gate indices whose `out_node` is that node (build once in `bind`, store `std::vector<std::vector<int>> nodeGateSources_`). Read pin drivers once at the top of `step`; only `gateOut_` changes across settle passes.

- [ ] **Step 4: Run to verify pass**
Run: `cmake --build build --target test_logic_net && ./build/test_logic_net.exe`
Expected: `logic_net_test: PASS`

- [ ] **Step 5: Commit**
```bash
git add src/logic_net.cpp test/logic_net_test.cpp
git commit -m "feat: logic_net gates + combinational settle

Co-Authored-By: Claude Opus 4.8 <noreply@anthropic.com>"
```

---

### Task 4: Hardened bind + node introspection

**Files:**
- Modify: `src/logic_net.cpp` (`bind` validation, accessors)
- Modify: `test/logic_net_test.cpp`

**Interfaces:**
- Consumes: everything above.
- Produces: `bind` returns `false` with `err = "unknown pin: <name>"` when the resolver returns a handle `< 0`; on success every endpoint has `handle >= 0` and `is_output` set, and `nodeGateSources_`/`level_`/`gateOut_`/warn-flags are sized. `node_count()` = `nodes_.size()`; `node_name(i)`/`node_level(i)` expose node state for the board view (`node_level` returns `LVL_X` before the first `step`).

- [ ] **Step 1: Write the failing test** — append and call:
```cpp
static void test_bind_unknown_pin() {
    ln::Net net; std::string err;
    net.parse("wire N ghost@0x99.NOPE\n", err);
    auto reject = [](void *, const char *, int *o) { *o = 0; return -1; };
    bool ok = net.bind(reject, nullptr, err);
    assert(!ok && err.find("unknown pin") != std::string::npos);
}
```

- [ ] **Step 2: Run to verify fail**
Run: `cmake --build build --target test_logic_net && ./build/test_logic_net.exe`
Expected: FAIL — `bind` currently accepts anything.

- [ ] **Step 3: Implement** the `< 0` handle check + `err` message in `bind`, and confirm the accessors return real node state. Ensure `bind` is idempotent-safe (sizes vectors once).

- [ ] **Step 4: Run to verify pass** — Expected: `logic_net_test: PASS`

- [ ] **Step 5: Commit**
```bash
git add src/logic_net.cpp test/logic_net_test.cpp
git commit -m "feat: logic_net bind validation + node introspection

Co-Authored-By: Claude Opus 4.8 <noreply@anthropic.com>"
```

---

### Task 5: Contract additions — pin_set + wiring_active + ABI bumps

**Files:**
- Modify: `include/i8085_io_plugin.h`

**Interfaces:**
- Produces (consumed by runtime + plugins):
  - Plugin API gains, appended at the END of `I8085IoPluginAPI`:
    ```cpp
    // Optional (plugin API v5). Deliver a resolved logic-net level to one of
    // this plugin's INPUT pins, named as in snapshot's pin buses (bit-indexed
    // for multi-pin buses, e.g. "CTS" or "A3"). level is 0, 1, or 2 (X/unknown).
    void (*pin_set)(void *ctx, const char *pin, UINT8 level);
    ```
  - Host API gains, appended at the END of `I8085HostAPI`:
    ```cpp
    // 1 once a --netlist is loaded: the logic net is authoritative for wired
    // pins (incl. the CPU interrupt lines). Chips that had a hardcoded effect
    // (e.g. MC6850 poking RST 7.5) must defer to the net when this is set.
    UINT32 wiring_active;
    ```
  - `#define I8085_IO_PLUGIN_ABI_VERSION 5u` and `#define I8085_HOST_API_VERSION 3u`.

- [ ] **Step 1: Edit the header** — bump both version macros; append `pin_set` to `I8085IoPluginAPI` (after `snapshot`); append `UINT32 wiring_active;` to `I8085HostAPI` (after `peer_snapshot`). Add the doc comments above verbatim.

- [ ] **Step 2: Rebuild everything + run an existing regression** (proves the ABI bump didn't break plugin loading — plugins report v5 via the macro, runtime still requires an exact match):
```bash
export PATH="/c/Users/Max/Downloads/mingw64/bin:$PATH"
cmake --build build
printf '\x3e\xaa\x32\x00\x01\x3a\x00\x01\xd3\xdf\x76' > build/mv.bin
./build/i8085-trace.exe -q -S -n 100 -l 0x0 -e 0x0 \
  --io-plugin=./build/memory.dll --io-plugin-config="base=0x0;size=0x8000;mode=eeprom;wren=off" \
  --io-plugin=./build/mc6850.dll --io-plugin-config="base=0xDE;txlog=build/o.bin" build/mv.bin >/dev/null 2>&1
xxd -p build/o.bin   # expect: 00  (eeprom veto still works under ABI v5)
```
Expected: `00`.

- [ ] **Step 3: Commit**
```bash
git add include/i8085_io_plugin.h
git commit -m "feat: plugin pin_set + host wiring_active (ABI v5 / host API v3)

Co-Authored-By: Claude Opus 4.8 <noreply@anthropic.com>"
```

---

### Task 6: Runtime net integration — own the Net, --netlist, plugin pin I/O

**Files:**
- Modify: `src/io_runtime.cpp`
- Modify: `include/i8085_io_runtime.h` (declare `io_runtime_load_netlist`)
- Modify: `src/main.cpp` (add `--netlist <file>`)
- Modify: `CMakeLists.txt` (add `src/logic_net.cpp` to the `i8085-trace` executable)

**Interfaces:**
- Consumes: `ln::Net`/`ln::Host` (Tasks 1-4); `snapshot` pin buses; `pin_set`, `wiring_active` (Task 5).
- Produces:
  - `int io_runtime_load_netlist(const char *path, char *errbuf, size_t errbuf_len);` — reads the file, `parse`s, `bind`s (resolver below), sets `gRuntime.wiring = true` and `runtime_host_api()`’s `wiring_active = 1`. Returns 0 on success, -1 on error (errbuf filled).
  - A pin resolver `int rt_resolve(void *ctx, const char *pin, int *is_output)` that parses `kind@base.Sub` (e.g. `mc6850@0xDE.IRQ`, `i8255@0x00.A3`), finds the loaded plugin whose `snapshot` `info.kind`+`info.base` match, locates the pin `Sub` in its buses (field name, and trailing digits = bit index; `is_output` = pin’s `is_input` bit == 0), and records a `PinHandle{plugin_idx, field, bit}` in `gRuntime.pinTable`, returning its index. `cpu.<Pin>` handles are added in Task 7 (here, `cpu.*` may resolve to -1 → that is fine until Task 7).
  - `ln::Host` impls: `rt_read_output(handle)` → call that plugin’s `snapshot` into a scratch `I8085StateField[32]`, find `field`, read `bus->hi_z`/`bus->level` at `bit` → `DRV_Z` / `DRV_0` / `DRV_1`. `rt_write_input(handle, level)` → call that plugin’s `pin_set(ctx, sub, level)` if non-null. `rt_warn` → `fprintf(stderr, "[wiring] %s\n", msg)`.
  - `io_runtime_on_step` calls `gRuntime.net.step(host)` **after** the plugin `on_step` loop, only when `gRuntime.wiring`.

- [ ] **Step 1: Write the failing integration test** — create `test/wiring_selfloop.sh` (a shell test; make it exit non-zero on failure):
```bash
#!/usr/bin/env bash
set -e
export PATH="/c/Users/Max/Downloads/mingw64/bin:$PATH"
cd "$(dirname "$0")/.."
cmake --build build >/dev/null
# Netlist: tie an i8255 Port A OUTPUT pin (A0) through a buffer to Port C INPUT
# pin C0, so writing PA bit0=1 should appear on PC bit0 read back by the CPU.
cat > build/self.net <<'EOF'
wire NA i8255@0x00.A0
gate buf B1 NA NC
wire NC i8255@0x00.C0
EOF
# Program: mode-set A output, C-lower input (0x81); PA=0x01; read PC -> UART.
#  3E 81 D3 03  3E 01 D3 00  DB 02 D3 DF  76
printf '\x3e\x81\xd3\x03\x3e\x01\xd3\x00\xdb\x02\xd3\xdf\x76' > build/self.bin
rm -f build/self.txt
./build/i8085-trace.exe -q -S -n 200 -l 0x0 -e 0x0 \
  --netlist build/self.net \
  --io-plugin=./build/i8255.dll  --io-plugin-config="base=0x0" \
  --io-plugin=./build/mc6850.dll --io-plugin-config="base=0xDE;txlog=build/self.txt" build/self.bin >/dev/null 2>&1
got=$(xxd -p build/self.txt)
echo "PC readback = $got (expect 01)"
[ "$got" = "01" ] || { echo FAIL; exit 1; }
echo PASS
```
> This exercises the full plugin→net→plugin path with no CPU pins, so it is testable before Task 7. It relies on i8255 `pin_set` (Task 9) — so this test is EXPECTED to fail until Task 9. Write it now; it first passes at Task 9. For THIS task, assert only that `--netlist` loads and binds without error (see Step 2 command).

- [ ] **Step 2: Run the load-only check to verify current failure**
Run:
```bash
export PATH="/c/Users/Max/Downloads/mingw64/bin:$PATH"
printf 'wire N i8255@0x00.A0\n' > build/min.net
./build/i8085-trace.exe --netlist build/min.net -q -S -n 1 -l 0 -e 0 \
  --io-plugin=./build/i8255.dll --io-plugin-config="base=0x0" build/self.bin 2>&1 | head
```
Expected (before implementing): an "unknown option --netlist" or similar error.

- [ ] **Step 3: Implement.** Add `#include "logic_net.h"` to `io_runtime.cpp`. Add to `IORuntimeState`: `ln::Net net; bool wiring=false; struct PinHandle{int plugin; std::string field; int bit;}; std::vector<PinHandle> pinTable;`. Implement `rt_resolve`/`rt_read_output`/`rt_write_input`/`rt_warn`, `io_runtime_load_netlist` (read file into a string, `parse`, `bind` with `rt_resolve`; on error fill errbuf, return -1; else set `wiring=true` and `runtime_host_api()`'s stored `wiring_active=1`). In `io_runtime_on_step`, after the plugin loop, `if (gRuntime.wiring) { ln::Host h{...}; gRuntime.net.step(h); }`. Declare `io_runtime_load_netlist` in `include/i8085_io_runtime.h`. In `src/main.cpp`, parse `--netlist <file>` (a new `getopt_long` entry or manual arg scan matching the existing `--io-plugin` handling) and call `io_runtime_load_netlist` after plugins are loaded; on failure print the error and exit non-zero. In `CMakeLists.txt` add `src/logic_net.cpp` to the `i8085-trace` `add_executable` source list.

- [ ] **Step 4: Run the load-only check to verify it now binds** (a valid pin resolves, an invalid one errors):
```bash
cmake --build build
./build/i8085-trace.exe --netlist build/min.net -q -S -n 1 -l 0 -e 0 \
  --io-plugin=./build/i8255.dll --io-plugin-config="base=0x0" build/self.bin 2>&1 | head
printf 'wire N i8255@0x00.ZZ9\n' > build/bad.net
./build/i8085-trace.exe --netlist build/bad.net -q -S -n 1 -l 0 -e 0 \
  --io-plugin=./build/i8255.dll --io-plugin-config="base=0x0" build/self.bin 2>&1 | head
```
Expected: first runs clean (exit 0); second prints an "unknown pin" error and exits non-zero.

- [ ] **Step 5: Commit**
```bash
git add src/io_runtime.cpp include/i8085_io_runtime.h src/main.cpp CMakeLists.txt test/wiring_selfloop.sh
git commit -m "feat: runtime logic-net integration (--netlist, plugin pin I/O)

Co-Authored-By: Claude Opus 4.8 <noreply@anthropic.com>"
```

---

### Task 7: CPU peer + interrupt drive + net synthetic peer

**Files:**
- Modify: `src/io_runtime.cpp`

**Interfaces:**
- Consumes: Task 6 runtime; `State8085` (has `r7_latch`, and RST6.5/5.5/TRAP/INTR/SID/SOD fields — confirm exact member names by grepping `i8085_cpu.h`/`i8085_state.*`; use the actual latch member for RST7.5, which existing MC6850 code sets as `state->r7_latch`).
- Produces:
  - `rt_resolve` also recognizes `cpu.<Pin>` (`RST7.5/RST6.5/RST5.5/TRAP/INTR/SID` inputs, `SOD` output) → `PinHandle{plugin=-1, field="cpu:<Pin>", bit=0}`. `rt_read_output` for `cpu.SOD` reads the CPU SOD latch; `rt_write_input` for a `cpu.*` input sets the matching CPU latch each step (e.g. `cpu.RST7.5`→`state->r7_latch = (level==LVL_1)`). An `LVL_X` input is treated as 0 (deasserted).
  - Two synthetic peers appended after the plugins in `host_peer_count`/`host_peer_snapshot`: index `plugins.size()+0` = `net` (one `I8085_FIELD_STR` field per node: name = node name, `s` = `"0"`/`"1"`/`"float"`/`"X"` — actually use `I8085_FIELD_U8` with value 0/1 and a separate STR for float/X; simplest: STR field per node whose `s` is one of `0/1/float/X`, computed from `node_level` + whether the node has a pull), `info.kind="net"`, `base=span=0`; index `plugins.size()+1` = `cpu` (BOOL/`I8085_FIELD_U8` fields `RST7.5,RST6.5,RST5.5,TRAP,INTR` from the latches), `info.kind="cpu"`. `host_peer_count` returns `plugins.size() + (wiring ? 2 : 0)`.

- [ ] **Step 1: Write the failing end-to-end test** — create `test/wiring_irq.sh`:
```bash
#!/usr/bin/env bash
set -e
export PATH="/c/Users/Max/Downloads/mingw64/bin:$PATH"
cd "$(dirname "$0")/.."
cmake --build build >/dev/null
# Netlist: MC6850 /IRQ (open-drain, active-low) -> pull-up -> inverter -> RST7.5
cat > build/irq.net <<'EOF'
pull IRQ_L up
wire IRQ_L mc6850@0xDE.IRQ
gate inv U1 IRQ_L RST_IN
wire RST_IN cpu.RST7.5
EOF
# Program: RST7.5 vector is 0x003C. Main enables MC6850 RX int + EI, then spins.
# The ISR writes 0xAA to the UART (txlog) as a sentinel, then HLT.
# Main @0x0000:
#   3E 15 D3 DE   ctrl=0x15 (RX int enabled, /RTS low)   -- CR bit7=1 enables RX int? use 0x95
#   FB            EI
#   00 C3 06 00   NOP; JMP 0x0006 (spin)   [loop @0x0006]
# NOTE: set CR so RX interrupt is enabled (bit7=1): use 0x95 not 0x15.
# ISR @0x003C: 3E AA D3 DF 76  (MVI A,0xAA; OUT 0xDF; HLT)
python3 - <<'PY'
b = bytearray(0x40)
b[0x00:0x06] = bytes([0x3E,0x95,0xD3,0xDE, 0xFB, 0x00])   # ctrl RXint, EI, NOP
b[0x06:0x09] = bytes([0xC3,0x06,0x00])                     # JMP 0x0006 spin
b[0x3C:0x41] = bytes([0x3E,0xAA,0xD3,0xDF,0x76])           # ISR: OUT 0xAA; HLT
open("build/irq.bin","wb").write(b)
PY
rm -f build/irq.txt
# Feed one RX byte (0x41) so the ACIA raises /IRQ.
./build/i8085-trace.exe -q -S -n 20000 -l 0x0 -e 0x0 \
  --netlist build/irq.net \
  --io-plugin=./build/mc6850.dll --io-plugin-config="base=0xDE;rx=41;txlog=build/irq.txt" build/irq.bin >/dev/null 2>&1
got=$(xxd -p build/irq.txt)
echo "ISR sentinel = $got (expect aa)"
[ "$got" = "aa" ] || { echo FAIL; exit 1; }
echo PASS
```
> This depends on the MC6850 open-drain `/IRQ` + `wiring_active` suppression (Task 8). Expected to first PASS at Task 8. For THIS task, verify the `cpu`/`net` peers render (Step 2).

- [ ] **Step 2: Run the peer-visibility check to verify current failure**
```bash
export PATH="/c/Users/Max/Downloads/mingw64/bin:$PATH"
cmake --build build
printf 'pull IRQ_L up\nwire IRQ_L mc6850@0xDE.IRQ\ngate inv U1 IRQ_L RST_IN\nwire RST_IN cpu.RST7.5\n' > build/irq.net
printf '\x00\xc3\x00\x00' > build/spin.bin
./build/i8085-trace.exe -q -S -n 5000 -l 0x0 -e 0x0 --netlist build/irq.net \
  --io-plugin=./build/mc6850.dll --io-plugin-config="base=0xDE" \
  --io-plugin=./build/boardview.dll --io-plugin-config="out=stdout;interval=1000" build/spin.bin 2>/dev/null | grep -E '\[net|\[cpu' | head
```
Expected (before implementing): no `[net]`/`[cpu]` lines. After Step 3: both appear.

- [ ] **Step 3: Implement** the `cpu.*` resolver branch, `rt_read_output`/`rt_write_input` CPU handling (map `cpu:RST7.5` → `state->r7_latch`, etc.; grep the real member names first), and the two synthetic peers in `host_peer_count`/`host_peer_snapshot`. Store static field-name arrays for the peers so the `const char*` names outlive the call. For the `net` peer, compute each node’s `s` from `net.node_level(i)` (and treat a no-driver/no-pull as `"float"`).

- [ ] **Step 4: Run the peer-visibility check to verify pass**
Run the Step 2 command again. Expected: `[net] IRQ_L=... RST_IN=...` and `[cpu] RST7.5=...` lines appear.

- [ ] **Step 5: Commit**
```bash
git add src/io_runtime.cpp
git commit -m "feat: cpu peer + net-driven interrupts + net/cpu board-view peers

Co-Authored-By: Claude Opus 4.8 <noreply@anthropic.com>"
```

---

### Task 8: MC6850 open-drain /IRQ + wiring_active suppression

**Files:**
- Modify: `plugins/mc6850.cpp`

**Interfaces:**
- Consumes: `pin_set`, `wiring_active` (Task 5); the CPU-interrupt drive (Task 7).
- Produces: MC6850 that (a) drives `/IRQ` open-drain — asserted (`rdrFull && rxIntEnabled`) → `busIRQ = {1, level=0, is_input=0, hi_z=0}`; else → `hi_z=1` (released); (b) in `on_step`, sets `state->r7_latch` **only when `!host->wiring_active`** (defer to the net otherwise); (c) implements `pin_set` for `CTS`/`DCD` (store into `ctsLevel`/`dcdLevel`).

- [ ] **Step 1: Write the failing test** — this is the Task 7 end-to-end test `test/wiring_irq.sh`. Run it:
```bash
bash test/wiring_irq.sh
```
Expected: FAIL (sentinel not `aa`) — the IRQ pin is push-pull and the direct poke double-drives / the net path isn't exercised.

- [ ] **Step 2: Confirm the failure reason** by dumping the `net` node during the run (optional): add `--io-plugin=boardview ... interval=200` and confirm `IRQ_L`/`RST_IN` toggle when the RX byte lands. Note whether `/IRQ` shows `z` before the byte and `0` after (it should after Step 3).

- [ ] **Step 3: Implement** in `plugins/mc6850.cpp`:
  - In `snapshot`, replace the `busIRQ` line with open-drain:
    ```cpp
    bool irqAsserted = (c->rdrFull && c->rxIntEnabled);
    c->busIRQ = irqAsserted ? I8085PinBus{1, 0u, 0u, 0u}   // drive low
                            : I8085PinBus{1, 0u, 0u, 1u};  // hi-Z (released)
    ```
  - In `on_step`, guard the interrupt poke:
    ```cpp
    if (c->rdrFull && c->rxIntEnabled && !(c->host && c->host->wiring_active))
        state->r7_latch = 1;
    ```
    (Only add the `wiring_active` guard; keep the existing condition otherwise.)
  - Add a `pin_set`:
    ```cpp
    static void pin_set(void *vctx, const char *pin, UINT8 level) {
        Ctx *c = (Ctx *)vctx;
        int lv = (level == 1) ? 1 : 0;              // X treated as 0
        if (!strcmp(pin, "CTS")) c->ctsLevel = lv;
        else if (!strcmp(pin, "DCD")) c->dcdLevel = lv;
    }
    ```
    (add `#include <cstring>` if needed) and wire `out_api->pin_set = pin_set;` in init.

- [ ] **Step 4: Run the end-to-end test to verify pass**
```bash
bash test/wiring_irq.sh
```
Expected: `ISR sentinel = aa (expect aa)` / `PASS`.

- [ ] **Step 5: No-netlist regression** — the same IRQ must still fire the OLD way without a netlist:
```bash
export PATH="/c/Users/Max/Downloads/mingw64/bin:$PATH"
./build/i8085-trace.exe -q -S -n 20000 -l 0x0 -e 0x0 \
  --io-plugin=./build/mc6850.dll --io-plugin-config="base=0xDE;rx=41;txlog=build/nn.txt" build/irq.bin >/dev/null 2>&1
xxd -p build/nn.txt   # expect: aa  (direct r7_latch path still works)
```
Expected: `aa`.

- [ ] **Step 6: Commit**
```bash
git add plugins/mc6850.cpp test/wiring_irq.sh
git commit -m "feat: MC6850 open-drain /IRQ, net-deferred interrupt, CTS/DCD pin_set

Co-Authored-By: Claude Opus 4.8 <noreply@anthropic.com>"
```

---

### Task 9: i8255 input pins via pin_set

**Files:**
- Modify: `plugins/i8255.cpp`

**Interfaces:**
- Consumes: `pin_set` (Task 5); the runtime pin I/O (Task 6).
- Produces: i8255 that accepts wired input-pin levels — `pin_set("A3", level)` sets bit 3 of the external input value for Port A (`inA`), `"B<n>"`→`inB`, `"C<n>"`→`inC`. A wired input pin thus supersedes the static `pa`/`pb`/`pc` config bit. Parsing: first char selects the port (`A`/`B`/`C`), the trailing digits are the bit index (0-7).

- [ ] **Step 1: Write the failing test** — the Task 6 `test/wiring_selfloop.sh`. Run it:
```bash
bash test/wiring_selfloop.sh
```
Expected: FAIL (PC readback not `01`) — i8255 ignores `pin_set`.

- [ ] **Step 2: Confirm failure** — Expected output shows `PC readback = 00` (or similar), not `01`.

- [ ] **Step 3: Implement** `pin_set` in `plugins/i8255.cpp`:
```cpp
static void pin_set(void *vctx, const char *pin, UINT8 level) {
    Ctx *c = (Ctx *)vctx;
    if (!pin || !pin[0]) return;
    int bit = atoi(pin + 1) & 7;
    UINT8 mask = (UINT8)(1u << bit);
    UINT8 val = (level == 1) ? mask : 0;
    switch (pin[0]) {
        case 'A': c->inA = (UINT8)((c->inA & ~mask) | val); break;
        case 'B': c->inB = (UINT8)((c->inB & ~mask) | val); break;
        case 'C': c->inC = (UINT8)((c->inC & ~mask) | val); break;
    }
}
```
(add `#include <cstdlib>` for `atoi` if not present) and set `out_api->pin_set = pin_set;` in init.

- [ ] **Step 4: Run the self-loop test to verify pass**
```bash
bash test/wiring_selfloop.sh
```
Expected: `PC readback = 01 (expect 01)` / `PASS`.

- [ ] **Step 5: Commit**
```bash
git add plugins/i8255.cpp
git commit -m "feat: i8255 input pins driven via pin_set (wired inputs)

Co-Authored-By: Claude Opus 4.8 <noreply@anthropic.com>"
```

---

### Task 10: Full regression sweep + ship

**Files:** none (verification + push)

**Interfaces:** consumes the whole feature.

- [ ] **Step 1: Run every test — unit + both integration + prior regressions**
```bash
export PATH="/c/Users/Max/Downloads/mingw64/bin:$PATH"
cmake --build build
./build/test_logic_net.exe                 # unit: expect "logic_net_test: PASS"
bash test/wiring_selfloop.sh               # expect PASS (01)
bash test/wiring_irq.sh                    # expect PASS (aa)
# prior behavior regressions (must be unchanged):
printf '\x3e\xaa\x32\x00\x01\x3a\x00\x01\xd3\xdf\x76' > build/mv.bin
./build/i8085-trace.exe -q -S -n 100 -l 0x0 -e 0x0 \
  --io-plugin=./build/memory.dll --io-plugin-config="base=0x0;size=0x8000;mode=eeprom;wren=off" \
  --io-plugin=./build/mc6850.dll --io-plugin-config="base=0xDE;txlog=build/o.bin" build/mv.bin >/dev/null 2>&1
xxd -p build/o.bin                         # expect 00 (memory veto)
```
Expected: all PASS / `00`.

- [ ] **Step 2: Clean the scratch build dir** (avoids the file-lock issue when committing): `rm -rf build 2>/dev/null || true` (it is gitignored; do NOT `git add` it).

- [ ] **Step 3: Push to main and watch CI**
```bash
git push "https://x-access-token:$(gh auth token)@github.com/maxgerhardt/tool-i8085-trace.git" main
sleep 5
gh run list --repo maxgerhardt/tool-i8085-trace --limit 1 --json databaseId,status --jq '.[0]'
```
Then watch: `gh run watch <id> --repo maxgerhardt/tool-i8085-trace --exit-status --interval 15`. Expected: success on all three OS. If the CI's `test/*.sh` or `build/` path assumptions differ from local (CI builds into `build/`), confirm the integration tests are either invoked by CI or left as local-only dev checks — do NOT block the ship on CI running the shell tests unless the workflow already does.

- [ ] **Step 4: Update the memory file** `multi-plugin-board-refactor.md` with the wiring feature (net engine, `--netlist`, open-drain IRQ, `pin_set`/`wiring_active` ABI v5/host v3) and mark the plan done.

---

## Self-Review

**Spec coverage:**
- Tri-state model (§1) → Task 8 (open-drain `/IRQ`), `hi_z` consumed in Task 6 `rt_read_output`. ✓
- Pin naming + `pin_set` contract (§2) → Task 5 (contract), Task 6 (`rt_resolve`/`kind@base.Sub`), Tasks 8/9 (`pin_set` impls). ✓
- Net engine resolution/gates/settle (§3) → Tasks 2/3; floating/conflict/oscillation warnings ✓.
- Netlist file (§4) → Task 1 parser + Task 6 `--netlist`/bind. ✓
- CPU peer + interrupt integration + `wiring_active` (§5) → Task 5 (flag), Task 7 (cpu pins/drive), Task 8 (suppression). ✓
- Board view synthetic peers (§6) → Task 7. ✓
- Testing (§7): resolution units (T2/T3), end-to-end IRQ→ISR (T7/T8), no-netlist regression (T8/T10), netlist errors (T4 bind, T6 unknown-pin). ✓

**Placeholder scan:** every code step contains real code; test programs are concrete byte sequences. The only deferred item is confirming exact `State8085` interrupt member names in Task 7 (instructed to grep `i8085_cpu.h` — `r7_latch` is already used by existing MC6850 code, so at minimum RST7.5 is known-good; RST6.5/5.5/TRAP/INTR/SID/SOD names must be confirmed there).

**Type consistency:** `ln::Drive`/`ln::Level`/`ln::Pull`/`ln::GateType`, `Host{read_output,write_input,warn}`, and `Net::{parse,bind,step,node_count,node_name,node_level}` are used identically across Tasks 1-7. `pin_set(void*,const char*,UINT8)` matches between the header (Task 5), runtime call (Task 6 `rt_write_input`), and impls (Tasks 8/9). `wiring_active` (UINT32 on `I8085HostAPI`) is set in Task 6 and read in Task 8. `PinHandle{plugin,field,bit}` is defined and used only within Task 6/7 runtime code.

**Note for the implementer:** Tasks 6/7/8/9 have interleaved test dependencies (the two integration tests are authored early but first pass later). Each task's Step 1 says explicitly when its test is expected to first pass. Follow the per-task "verify fail / verify pass" using the checks named in that task, not only the shared shell scripts.
