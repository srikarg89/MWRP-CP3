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

        Node(int id, std::vector<AgentState> a, boost::dynamic_bitset<> s, int c, int h, int n){
            node_id = id;
            agents = a;
            seen = s;
            cost = c;
            heuristic = h;
            num_seen = n;
            f_value = c + h;
        }

        bool operator>(const Node& rhs) const
        {
            return std::tie(f_value, heuristic) > std::tie(rhs.f_value, rhs.heuristic);
        }

    };

    int get_bfs_heuristic();
    int get_singleton_heuristic(int node_map_idx, const boost::dynamic_bitset<>& seen, const std::vector<std::vector<int>>& min_dist_to_see);
    int get_mst_heuristic(const DisjointGraph& disjoint_graph);
    // int get_tsp_heuristic(const DisjointGraph& disjoint_graph);
    int get_heuristic(HeuristicType heuristic_type, const Map& map, std::vector<AgentState> agent_states, const boost::dynamic_bitset<>& seen, const Lookup& lookup);
    std::vector<Node> get_neighbors(Node& node, const Map& map, MovementType movement, const Lookup& lookup, HeuristicType heuristic_type, int last_id_assigned, bool jump_to_frontier, std::vector<Node>& all_nodes);
    std::vector<std::vector<Position>> run_watchman(std::vector<Position> starts, LOSType los, const Map& map, MovementType movement, HeuristicType heuristic_type, bool jump_to_frontier);
}
