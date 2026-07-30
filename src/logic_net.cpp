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

    err = "";
    return true;
}

void Net::step(const Host &host) {
    // For each node, compute its level based on output endpoints and pull
    for (int node_idx = 0; node_idx < (int)nodes_.size(); ++node_idx) {
        bool has0 = false;
        bool has1 = false;

        // Read every bound OUTPUT endpoint's Drive
        for (const auto &ep : endpoints_) {
            if (ep.node == node_idx && ep.is_output && ep.handle >= 0) {
                Drive drv = host.read_output(host.ctx, ep.handle);
                if (drv == DRV_0) {
                    has0 = true;
                } else if (drv == DRV_1) {
                    has1 = true;
                }
                // Ignore DRV_Z
            }
        }

        // Apply resolution rule
        Level lvl;
        if (has0 && has1) {
            // Conflict
            lvl = LVL_X;
            if (!warnedConflict_[node_idx]) {
                host.warn(host.ctx, "conflict on node");
                warnedConflict_[node_idx] = true;
            }
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

        level_[node_idx] = lvl;
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
