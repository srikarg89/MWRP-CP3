#pragma once

#include <cstdlib>
#include <boost/dynamic_bitset.hpp>
#include "shared.hpp"

// Header file for the single-agent watchman algorithm.

namespace watchman {

    struct AgentState{
        Position pos;
        bool terminated;
        int cost;

        AgentState(Position p, bool t, int c) : pos(p), terminated(t), cost(c) {}
    };

    struct Node {
        int node_id;
        std::vector<AgentState> agents;
        boost::dynamic_bitset<> seen;
        int cost;
        int heuristic;
        int num_seen;
        int f_value;
        bool is_lazy;

        Node(int id, std::vector<AgentState> a, boost::dynamic_bitset<> s, int c, int f, int n){
            node_id = id;
            agents = a;
            seen = s;
            cost = c;
            heuristic = f - c;
            num_seen = n;
            f_value = f;
            is_lazy = true;
        }

        void update_f_value(int new_f_value){
            f_value = new_f_value;
            heuristic = f_value - cost;
            is_lazy = false;
        }

        bool operator>(const Node& rhs) const
        {
            // int weighted_f = (int)(cost + heuristic * 1.5);
            // int rhs_weighted_f = (int)(rhs.cost + rhs.heuristic * 1.5);
            // return std::tie(weighted_f, heuristic) > std::tie(rhs_weighted_f, rhs.heuristic);
            return std::tie(f_value, heuristic) > std::tie(rhs.f_value, rhs.heuristic);
        }

    };

    std::vector<std::vector<Position>> run_watchman(std::vector<Position> starts, const ScenarioConfig& scenario_config, const SolverConfig& solver_config);
}
