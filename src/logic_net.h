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

    int nodeIndex(const std::string &name);
};

} // namespace ln
