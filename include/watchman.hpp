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


    ////////////////////////////////
    ///////// Forms of LOS /////////
    ////////////////////////////////

    // Four-way line of sight (rook movements).
    inline std::vector<Position> four_way_LOS(Position pos, const Map& map){
        int dx[] = {-1, 1, 0, 0};
        int dy[] = {0, 0, -1, 1};

        std::vector<Position> los;
        los.push_back(pos);

        for(int i = 0; i < 4; i++){
            Position curr = pos;

            while(true){
                curr = curr.add(dx[i], dy[i]);
                if(map.check_obstacle(curr)){
                    break;
                }
                los.push_back(curr);
            }
        }

        return los;
    }

    // Eight-way line of sight (queen movements).
    inline std::vector<Position> eight_way_LOS(Position pos, const Map& map){
        int dx[] = {-1, 1, 0, 0, -1, -1, 1, 1};
        int dy[] = {0, 0, -1, 1, -1, 1, -1, 1};

        std::vector<Position> los;
        los.push_back(pos);

        for(int i = 0; i < 8; i++){
            Position curr = pos;

            while(true){
                curr = curr.add(dx[i], dy[i]);
                if(map.check_obstacle(curr)){
                    break;
                }
                los.push_back(curr);
            }
        }

        return los;
    }

    // Bresham line of sight. Very naive approach. O(N^3) ://
    inline bool pairwiseBresLos(Position pos1, Position pos2, const Map& map){
        int dx = std::abs(pos2.x - pos1.x);
        int dy = std::abs(pos2.y - pos1.y);
        int sx = (pos1.x < pos2.x) ? 1 : -1;
        int sy = (pos1.y < pos2.y) ? 1 : -1;
        int err = dx - dy;

        Position curr = pos1;
        while(true){
            if(map.check_obstacle(curr)){
                return false;
            }
            if(curr.equals(pos2)){
                return true;
            }

            int e2 = 2 * err;
            if(e2 > -dy){
                err -= dy;
                curr = curr.add(sx, 0);
            }
            if(e2 < dx){
                err += dx;
                curr = curr.add(0, sy);
            }
        }
        return true;
    }


    inline std::vector<Position> bresLOS(Position pos, const Map& map){
        std::vector<Position> los;
        for(int x = 0; x < map.x_size; x++){
            for(int y = 0; y < map.y_size; y++){
                Position candidate = {x, y};
                if(pairwiseBresLos(pos, candidate, map)){
                    los.push_back(candidate);
                }
            }
        }

        return los;
    }

    enum HeuristicType {
        BFS,
        SINGLETON,
        MST,
        TSP
    };

    struct Lookup {
        std::vector<std::vector<Position>> los;
        std::vector<std::vector<int>> apsp;
        std::vector<std::vector<int>> min_dist_to_see;
        std::vector<std::vector<int>> pivot_cell_dists;
        std::vector<std::vector<int>> pivot_pivot_dists;
        std::vector<int> sorted_los_order;
    };

    struct DisjointGraph {
        std::vector<int> nodes;
        std::vector<std::vector<int>> edge_costs;
    };

    int add_los_to_seen(boost::dynamic_bitset<>& seen, const std::vector<Position>& los, const Map& map);
    int get_bfs_heuristic();
    int get_singleton_heuristic(int node_map_idx, const boost::dynamic_bitset<>& seen, const std::vector<std::vector<int>>& min_dist_to_see);
    int get_mst_heuristic(const Lookup& lookup, int node_map_idx, const boost::dynamic_bitset<>& seen);
    int get_heuristic(HeuristicType heuristic_type, int node_map_idx, const boost::dynamic_bitset<>& seen, const Lookup& lookup);
    std::vector<Node> get_neighbors(Node& node, const Map& map, MovementType movement, const Lookup& lookup, HeuristicType heuristic_type, int last_id_assigned);
    void precompute_lookup(Lookup& lookup, LOSType los, const Map& map, MovementType movement, HeuristicType heuristic_type);
    DisjointGraph compute_disjoint_graph(const Lookup& lookup, int agent_map_idx, const boost::dynamic_bitset<>& seen);
    std::vector<Position> run_watchman(Position start, LOSType los, const Map& map, MovementType movement, HeuristicType heuristic_type);
}
