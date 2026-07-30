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
