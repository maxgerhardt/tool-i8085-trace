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
    Fake f = {}; ln::Host host{&f, fake_read, fake_write, fake_warn};
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

static void test_inverter_chain() {
    ln::Net net; std::string err;
    // O drives IN; inverter U1: IN -> MID; inverter U2: MID -> OUTN; input I on OUTN.
    net.parse("wire IN O\ngate inv U1 IN MID\ngate inv U2 MID OUTN\nwire OUTN I\n", err);
    int next = 0; net.bind(seq_resolve, &next, err);   // O=h0(out), I=h1(in)
    Fake f = {}; ln::Host host{&f, fake_read, fake_write, fake_warn};
    f.drv[0] = ln::DRV_0; net.step(host);
    assert(f.in[1] == ln::LVL_0); // 0 -> ~0=1 -> ~1=0
    assert(f.warns == 0);  // No spurious conflict warnings from gate-forced-X
    f.drv[0] = ln::DRV_1; net.step(host);
    assert(f.in[1] == ln::LVL_1);
    assert(f.warns == 0);  // Still no warnings
}

static void test_and_gate() {
    ln::Net net; std::string err;
    net.parse("wire A O0\nwire B O1\ngate and G A B Y\nwire Y I\n", err);
    int next = 0; net.bind(seq_resolve, &next, err);   // O0=h0,O1=h1(out), I=h2(in)
    Fake f = {}; ln::Host host{&f, fake_read, fake_write, fake_warn};
    f.drv[0]=ln::DRV_1; f.drv[1]=ln::DRV_1; net.step(host); assert(f.in[2]==ln::LVL_1);
    f.drv[1]=ln::DRV_0; net.step(host);                    assert(f.in[2]==ln::LVL_0);
}

static void test_gate_conflict_detection() {
    ln::Net net; std::string err;
    // Two gate outputs driving the same node with opposite values: Y1=1, Y2=0
    // gate buf U1 A Y; gate inv U2 A Y (both drive Y)
    net.parse("wire A O\ngate buf U1 A Y\ngate inv U2 A Y\nwire Y I\n", err);
    int next = 0; net.bind(seq_resolve, &next, err);   // O=h0(out), I=h1(in)
    Fake f = {}; ln::Host host{&f, fake_read, fake_write, fake_warn};
    f.drv[0] = ln::DRV_1; net.step(host);
    // U1 (buf) outputs 1, U2 (inv) outputs 0 -> real conflict
    assert(f.in[1] == ln::LVL_X && f.warns > 0);  // Real conflict detected and warned
}

int main() {
    test_parse_basic();
    test_parse_error();
    test_wired_and_pullup();
    test_conflict_and_float();
    test_inverter_chain();
    test_and_gate();
    test_gate_conflict_detection();
    printf("logic_net_test: PASS\n");
    return 0;
}
