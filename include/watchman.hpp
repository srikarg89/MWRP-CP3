#pragma once

#include <cstdlib>
#include <boost/dynamic_bitset.hpp>
#include "shared.hpp"

// Header file for the single-agent watchman algorithm.

namespace watchman {

    struct Node {
        int node_id;
        Position pos;
        boost::dynamic_bitset<> seen;
        int cost;
        int heuristic;
        int num_seen;
        int f_value;

        Node(int id, Position p, boost::dynamic_bitset<> s, int c, int h, int n){
            node_id = id;
            pos = p;
            seen = s;
            cost = c;
            heuristic = h;
            num_seen = n;
            f_value = c + h;
        }

        bool operator>(const Node& rhs) const
        {
            return f_value > rhs.f_value;
        }

    };

    int get_bfs_heuristic();
    int get_singleton_heuristic(int node_map_idx, const boost::dynamic_bitset<>& seen, const std::vector<std::vector<int>>& min_dist_to_see);
    int get_mst_heuristic(const Lookup& lookup, int node_map_idx, const boost::dynamic_bitset<>& seen);
    int get_heuristic(HeuristicType heuristic_type, int node_map_idx, const boost::dynamic_bitset<>& seen, const Lookup& lookup);
    std::vector<Node> get_neighbors(Node& node, const Map& map, MovementType movement, const Lookup& lookup, HeuristicType heuristic_type, int last_id_assigned, bool jump_to_frontier);
    std::vector<Position> run_watchman(Position start, LOSType los, const Map& map, MovementType movement, HeuristicType heuristic_type, bool jump_to_frontier);
}
