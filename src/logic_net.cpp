#include "logic_net.h"
#include <sstream>
#include <cctype>

namespace ln {

int Net::nodeIndex(const std::string &name) {
    // Find existing node
    for (size_t i = 0; i < nodes_.size(); ++i) {
        if (nodes_[i].name == name) {
            return (int)i;
        }
    }
    // Create new node
    nodes_.push_back(Node{name, PULL_NONE});
    return (int)nodes_.size() - 1;
}

Level Net::evalGate(const Gate &gate, const std::vector<Level> &nodeValues) const {
    // Evaluate gate output based on input node values
    // Gate truth (X-aware): INV(a):0→1,1→0,X→X. BUF(a):a.
    // AND: any 0→0; else any X→X; else 1.
    // OR: any 1→1; else any X→X; else 0.
    // NAND: any 0→1; else any X→X; else 0.
    // NOR: any 1→0; else any X→X; else 1.

    switch (gate.type) {
        case G_INV: {
            // Single input inverter
            if (gate.in_nodes.size() != 1) return LVL_X;
            Level in_val = nodeValues[gate.in_nodes[0]];
            if (in_val == LVL_0) return LVL_1;
            if (in_val == LVL_1) return LVL_0;
            return LVL_X;  // in_val == LVL_X
        }
        case G_BUF: {
            // Single input buffer
            if (gate.in_nodes.size() != 1) return LVL_X;
            return nodeValues[gate.in_nodes[0]];
        }
        case G_AND: {
            // AND: any 0→0; else any X→X; else 1
            bool has_x = false;
            for (int in_node : gate.in_nodes) {
                Level in_val = nodeValues[in_node];
                if (in_val == LVL_0) return LVL_0;
                if (in_val == LVL_X) has_x = true;
            }
            if (has_x) return LVL_X;
            return LVL_1;
        }
        case G_OR: {
            // OR: any 1→1; else any X→X; else 0
            bool has_x = false;
            for (int in_node : gate.in_nodes) {
                Level in_val = nodeValues[in_node];
                if (in_val == LVL_1) return LVL_1;
                if (in_val == LVL_X) has_x = true;
            }
            if (has_x) return LVL_X;
            return LVL_0;
        }
        case G_NAND: {
            // NAND: any 0→1; else any X→X; else 0
            bool has_x = false;
            for (int in_node : gate.in_nodes) {
                Level in_val = nodeValues[in_node];
                if (in_val == LVL_0) return LVL_1;
                if (in_val == LVL_X) has_x = true;
            }
            if (has_x) return LVL_X;
            return LVL_0;
        }
        case G_NOR: {
            // NOR: any 1→0; else any X→X; else 1
            bool has_x = false;
            for (int in_node : gate.in_nodes) {
                Level in_val = nodeValues[in_node];
                if (in_val == LVL_1) return LVL_0;
                if (in_val == LVL_X) has_x = true;
            }
            if (has_x) return LVL_X;
            return LVL_1;
        }
        default:
            return LVL_X;
    }
}

bool Net::parse(const std::string &text, std::string &err) {
    std::istringstream iss(text);
    std::string line;
    int line_num = 0;

    while (std::getline(iss, line)) {
        ++line_num;

        // Trim leading/trailing whitespace
        size_t start = 0;
        while (start < line.size() && std::isspace(line[start])) ++start;
        while (line.size() > start && std::isspace(line.back())) line.pop_back();
        line = line.substr(start);

        // Skip empty lines and comments
        if (line.empty() || line[0] == '#') {
            continue;
        }

        // Tokenize on whitespace
        std::vector<std::string> tokens;
        std::istringstream token_stream(line);
        std::string token;
        while (token_stream >> token) {
            tokens.push_back(token);
        }

        if (tokens.empty()) {
            continue;
        }

        const std::string &directive = tokens[0];

        if (directive == "pull") {
            if (tokens.size() != 3) {
                err = "line " + std::to_string(line_num) + ": pull requires node and up/down";
                return false;
            }
            int idx = nodeIndex(tokens[1]);
            const std::string &pull_dir = tokens[2];
            if (pull_dir == "up") {
                nodes_[idx].pull = PULL_UP;
            } else if (pull_dir == "down") {
                nodes_[idx].pull = PULL_DOWN;
            } else {
                err = "line " + std::to_string(line_num) + ": pull direction must be up or down";
                return false;
            }
        } else if (directive == "wire") {
            if (tokens.size() != 3) {
                err = "line " + std::to_string(line_num) + ": wire requires node and pin";
                return false;
            }
            int idx = nodeIndex(tokens[1]);
            endpoints_.push_back(Endpoint{idx, tokens[2], -1, false});
        } else if (directive == "gate") {
            if (tokens.size() < 4) {
                err = "line " + std::to_string(line_num) + ": gate requires type, name, and at least one input and one output";
                return false;
            }

            const std::string &gate_type_str = tokens[1];
            GateType gate_type;

            if (gate_type_str == "inv") {
                gate_type = G_INV;
            } else if (gate_type_str == "buf") {
                gate_type = G_BUF;
            } else if (gate_type_str == "and") {
                gate_type = G_AND;
            } else if (gate_type_str == "or") {
                gate_type = G_OR;
            } else if (gate_type_str == "nand") {
                gate_type = G_NAND;
            } else if (gate_type_str == "nor") {
                gate_type = G_NOR;
            } else {
                err = "line " + std::to_string(line_num) + ": unknown gate type '" + gate_type_str + "'";
                return false;
            }

            // tokens[2] is the gate name (not used in storage currently)
            // tokens[3..n-1] are input node names
            // tokens[n] is the output node name
            std::vector<int> in_nodes;
            for (size_t i = 3; i < tokens.size() - 1; ++i) {
                in_nodes.push_back(nodeIndex(tokens[i]));
            }
            int out_node = nodeIndex(tokens.back());

            gates_.push_back(Gate{gate_type, in_nodes, out_node});
        } else {
            err = "line " + std::to_string(line_num) + ": unknown directive '" + directive + "'";
            return false;
        }
    }

    err = "";
    return true;
}

bool Net::bind(int (*resolve)(void *ctx, const char *pin, int *is_output),
               void *ctx, std::string &err) {
    // Walk endpoints_, call resolve for each pin, store handle/is_output
    for (auto &ep : endpoints_) {
        int is_output_flag = 0;
        int handle = resolve(ctx, ep.pin.c_str(), &is_output_flag);
        if (handle < 0) {
            err = "failed to resolve pin " + ep.pin;
            return false;
        }
        ep.handle = handle;
        ep.is_output = (is_output_flag != 0);
    }

    // Initialize level and warning tracking vectors
    level_.resize(nodes_.size(), LVL_X);
    warnedConflict_.resize(nodes_.size(), false);
    warnedFloat_.resize(nodes_.size(), false);

    // Initialize gate outputs to LVL_X
    gateOut_.resize(gates_.size(), LVL_X);

    // Precompute per-node the list of gate indices whose out_node is that node
    nodeGateSources_.resize(nodes_.size());
    for (size_t gate_idx = 0; gate_idx < gates_.size(); ++gate_idx) {
        int out_node = gates_[gate_idx].out_node;
        if (out_node >= 0 && out_node < (int)nodes_.size()) {
            nodeGateSources_[out_node].push_back((int)gate_idx);
        }
    }

    // Initialize oscillation warning flag
    warnedOscillation_ = false;

    err = "";
    return true;
}

void Net::step(const Host &host) {
    // Guard: vectors must be properly sized by bind()
    if ((int)level_.size() != (int)nodes_.size()) return;
    if ((int)warnedConflict_.size() != (int)nodes_.size()) return;
    if ((int)warnedFloat_.size() != (int)nodes_.size()) return;
    if ((int)gateOut_.size() != (int)gates_.size()) return;
    if ((int)nodeGateSources_.size() != (int)nodes_.size()) return;

    // Read pin drivers ONCE at top of step(); cache per output endpoint
    std::vector<Drive> pin_drivers(endpoints_.size());
    for (size_t i = 0; i < endpoints_.size(); ++i) {
        if (endpoints_[i].is_output && endpoints_[i].handle >= 0) {
            pin_drivers[i] = host.read_output(host.ctx, endpoints_[i].handle);
        } else {
            pin_drivers[i] = DRV_Z;  // Dummy value for non-output endpoints
        }
    }

    // Settle loop: up to 16 passes
    const int MAX_PASSES = 16;
    bool oscillating = false;
    for (int pass = 0; pass < MAX_PASSES; ++pass) {
        bool level_changed = false;
        bool gateOut_changed = false;

        // (a) Resolve all nodes
        for (int node_idx = 0; node_idx < (int)nodes_.size(); ++node_idx) {
            bool has0 = false;
            bool has1 = false;
            bool gateX = false;

            // Read every bound OUTPUT endpoint's Drive for this node
            for (size_t ep_idx = 0; ep_idx < endpoints_.size(); ++ep_idx) {
                const auto &ep = endpoints_[ep_idx];
                if (ep.node == node_idx && ep.is_output && ep.handle >= 0) {
                    Drive drv = pin_drivers[ep_idx];
                    if (drv == DRV_0) {
                        has0 = true;
                    } else if (drv == DRV_1) {
                        has1 = true;
                    }
                    // Ignore DRV_Z
                }
            }

            // Treat each gate driving this node as a driver
            // Only settled gate outputs (LVL_0/LVL_1) contribute to has0/has1
            for (int gate_idx : nodeGateSources_[node_idx]) {
                Level gate_out = gateOut_[gate_idx];
                if (gate_out == LVL_0) {
                    has0 = true;
                } else if (gate_out == LVL_1) {
                    has1 = true;
                } else if (gate_out == LVL_X) {
                    // Gate is forcing X: track separately from real conflict
                    gateX = true;
                }
            }

            // Apply resolution rule with proper precedence
            Level lvl;
            if (has0 && has1) {
                // Real conflict: two settled drivers pushing opposite values
                lvl = LVL_X;
                if (!warnedConflict_[node_idx]) {
                    host.warn(host.ctx, "conflict on node");
                    warnedConflict_[node_idx] = true;
                }
            } else if (gateX) {
                // Gate forcing X (no real conflict)
                lvl = LVL_X;
            } else if (has0) {
                lvl = LVL_0;
            } else if (has1) {
                lvl = LVL_1;
            } else {
                // Neither driver is active, check pull
                Pull pull = nodes_[node_idx].pull;
                if (pull == PULL_UP) {
                    lvl = LVL_1;
                } else if (pull == PULL_DOWN) {
                    lvl = LVL_0;
                } else {
                    // PULL_NONE - floating
                    lvl = LVL_X;
                    if (!warnedFloat_[node_idx]) {
                        host.warn(host.ctx, "floating node");
                        warnedFloat_[node_idx] = true;
                    }
                }
            }

            if (level_[node_idx] != lvl) {
                level_[node_idx] = lvl;
                level_changed = true;
            }
        }

        // (b) Recompute each gate from its input nodes
        for (size_t gate_idx = 0; gate_idx < gates_.size(); ++gate_idx) {
            Level new_out = evalGate(gates_[gate_idx], level_);
            if (gateOut_[gate_idx] != new_out) {
                gateOut_[gate_idx] = new_out;
                gateOut_changed = true;
            }
        }

        // Stop if neither changed
        if (!level_changed && !gateOut_changed) {
            break;
        }

        // Check if we're on the last pass
        if (pass == MAX_PASSES - 1) {
            oscillating = true;
        }
    }

    // Warn once if oscillation detected
    if (oscillating && !warnedOscillation_) {
        host.warn(host.ctx, "oscillation");
        warnedOscillation_ = true;
    }

    // Deliver resolved levels to INPUT endpoints
    for (const auto &ep : endpoints_) {
        if (!ep.is_output && ep.handle >= 0) {
            host.write_input(host.ctx, ep.handle, level_[ep.node]);
        }
    }
}

int Net::node_count() const {
    return (int)nodes_.size();
}

const std::string &Net::node_name(int i) const {
    static const std::string empty_str;
    if (i < 0 || i >= (int)nodes_.size()) {
        return empty_str;
    }
    return nodes_[i].name;
}

Level Net::node_level(int i) const {
    if (i < 0 || i >= (int)nodes_.size()) {
        return LVL_X;
    }
    if (i >= (int)level_.size()) {
        return LVL_X;
    }
    return level_[i];
}

} // namespace ln
