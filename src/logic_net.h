#pragma once

#include <string>
#include <vector>

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
    // Optional. Current direction of a pin: nonzero = output/driver, 0 = input.
    // Called each step so a pin whose owner reprograms its direction at runtime
    // (e.g. an 8255 port) is routed live -- read as a driver while it is an
    // output, delivered to as a receiver while it is an input. NULL keeps the
    // bind-time direction fixed (used by unit tests with static directions).
    int   (*is_output)(void *ctx, int handle);
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

private:
    struct Node {
        std::string name;
        Pull pull;
    };

    struct Endpoint {
        int node;
        std::string pin;
        int handle = -1;
        bool is_output = false;
    };

    struct Gate {
        GateType type;
        std::vector<int> in_nodes;
        int out_node;
    };

    std::vector<Node> nodes_;
    std::vector<Endpoint> endpoints_;
    std::vector<Gate> gates_;
    std::vector<Level> level_;                           // Current level of each node
    std::vector<bool> warnedConflict_;                   // Warning tracking for conflicts
    std::vector<bool> warnedFloat_;                      // Warning tracking for floating
    std::vector<Level> gateOut_;                         // Output level of each gate
    std::vector<std::vector<int>> nodeGateSources_;      // Gate indices driving each node
    bool warnedOscillation_ = false;                      // Warning tracking for oscillation

    int nodeIndex(const std::string &name);
    Level evalGate(const Gate &gate, const std::vector<Level> &nodeValues) const;
};

} // namespace ln
