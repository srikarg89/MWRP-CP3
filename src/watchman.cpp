#include "shared.hpp"
#include "watchman.hpp"
#include "pathfinding.hpp"
#include <queue>
#include <unordered_map>

namespace watchman {
    // Returns the number of new squares marked as seen.
    int add_los_to_seen(std::vector<bool>& seen, const std::vector<Position>& los, const Map& map){
        int count = 0;
        for(Position pos : los){
            int map_idx = map.get_map_idx(pos);
            if(!seen[map_idx]){
                count += 1;
            }
            seen[map_idx] = true;
        }
        return count;
    }

    int get_bfs_heuristic(){
        printf("\tUsing BFS heuristic: %d\n", 1);
        return 1;
    }

    int get_singleton_heuristic(int node_map_idx, const std::vector<bool>& seen, const std::vector<std::vector<int>>& min_dist_to_see){
        int heuristic = 0;
        for(int i = 0; i < seen.size(); i++){
            if(!seen[i]) {
                heuristic = std::max(heuristic, min_dist_to_see[node_map_idx][i]);
            }
        }
        printf("\tComputing heuristic for node: %d is %d\n", node_map_idx, heuristic);
        return heuristic;
    }

    int get_heuristic(HeuristicType heuristic_type, int node_map_idx, const std::vector<bool>& seen, const std::vector<std::vector<int>>& min_dist_to_see){
        switch(heuristic_type){
            case BFS:
                return get_bfs_heuristic();
            case SINGLETON:
                return get_singleton_heuristic(node_map_idx, seen, min_dist_to_see);
            default:
                printf("UNKNOWN HEURISTIC TYPE ???\n");
                assert(false);
        }
        assert(false);
        return -1;
    }

    // TODO: Fix the cost calculation if we do weighted edges instead of just saying "node.cost + 1".
    std::vector<Node> get_neighbors(Node& node, const Map& map, MovementType movement, const Lookup& lookup, HeuristicType heuristic_type, int last_id_assigned){
        std::vector<Position> neighbors = map.get_neighbors(node.pos, movement);
        std::vector<Node> neighbor_nodes;
        for(Position nbr : neighbors){
            std::vector<bool> nbr_seen = node.seen;
            int new_squares_seen = add_los_to_seen(nbr_seen, lookup.los[map.get_map_idx(nbr)], map);
            int nbr_heuristic = get_heuristic(heuristic_type, map.get_map_idx(nbr), nbr_seen, lookup.min_dist_to_see);
            last_id_assigned += 1;
            neighbor_nodes.push_back(Node(last_id_assigned, nbr, nbr_seen, node.cost + 1, nbr_heuristic, node.num_seen + new_squares_seen));
        }
        return neighbor_nodes;
    }

    // Inputs: Agent starting position, LOS type, map.
    // TODO: Add in radius for LOS.
    // Output: Optimal path.
    std::vector<Position> run_watchman(Position start, LOSType los, const Map& map, MovementType movement, HeuristicType heuristic_type){
        // Precompute the LOS Lookup and the All Pairs Shortest Path (APSP)
        Lookup lookup;
        printf("Running watchman method!\n");
        for(int map_idx = 0; map_idx < map.x_size * map.y_size; map_idx++){
            Position pos = map.get_pos_from_map_idx(map_idx);
            if(map.check_obstacle(pos)){
                lookup.los.push_back({});
                std::vector<int> infinite_distances(map.x_size * map.y_size, INT_MAX);
                lookup.apsp.push_back(infinite_distances);
            } else {
                switch(los){
                    case FOUR_WAY_LOS:
                        lookup.los.push_back(four_way_LOS(pos, map));
                        break;
                    case EIGHT_WAY_LOS:
                        lookup.los.push_back(eight_way_LOS(pos, map));
                        break;
                    case BRES_LOS:
                        lookup.los.push_back(bresLOS(pos, map));
                        break;                        
                }

                std::vector<int> distances = pathfinding::get_bfs_distances(pos, movement, map);
                lookup.apsp.push_back(distances);

            }
            printf("Position: %s\n\t%s\n", pos.toString().c_str(), pos_array_to_string(lookup.los.back()).c_str());
        }

        printf("\n\n");

        // Precompute the Singleton heuristic helper lookup table.
        for(int map_idx = 0; map_idx < map.x_size * map.y_size; map_idx++){
            std::vector<int> min_dists(map.x_size * map.y_size, INT_MAX);
            lookup.min_dist_to_see.push_back(min_dists);
        }

        for(int g_map_idx = 0; g_map_idx < map.x_size * map.y_size; g_map_idx++){
            const std::vector<Position>& los = lookup.los[g_map_idx];
            for(Position los_pos : los){
                int los_idx = map.get_map_idx(los_pos);
                for(int s_map_idx = 0; s_map_idx < map.x_size * map.y_size; s_map_idx++){
                    lookup.min_dist_to_see[s_map_idx][g_map_idx] = std::min(lookup.min_dist_to_see[s_map_idx][g_map_idx], lookup.apsp[s_map_idx][los_idx]);
                }
            }
        }

        // Initialize search data structures.
        std::priority_queue<Node, std::vector<Node>, std::greater<Node>> queue;
        std::unordered_map<int, int> pred_lookup;
        std::unordered_map<int, Position> id_lookup;

        // Setup the starting variable. Mark all obstacles as seen.
        std::vector<bool> start_seen(map.x_size * map.y_size, false);
        int num_start_seen = 0;
        for(int map_idx = 0; map_idx < map.x_size * map.y_size; map_idx++){
            if(map.check_obstacle(map.get_pos_from_map_idx(map_idx))){
                start_seen[map_idx] = true;
                num_start_seen += 1;
            }
        }
        num_start_seen += add_los_to_seen(start_seen, lookup.los[map.get_map_idx(start)], map);
        printf("Start idx: %d\n", map.get_map_idx(start));
        int start_heuristic = get_heuristic(heuristic_type, map.get_map_idx(start), start_seen, lookup.min_dist_to_see);
        queue.push(Node(/* id = */ 0, start, start_seen, /* cost = */ 0, start_heuristic, num_start_seen));

        // assert(false);

        int num_expanded = 0;
        int last_id_assigned = 0;

        while(!queue.empty()){
            Node curr = queue.top();
            queue.pop();

            id_lookup[curr.node_id] = curr.pos;

            num_expanded += 1;
            printf("Expanding node %d. Loc: %s, cost: %d, heuristic: %d, num seen: %d\n", num_expanded, curr.pos.toString().c_str(), curr.cost, curr.heuristic, curr.num_seen);

            if(curr.num_seen == map.x_size * map.y_size){
                printf("Goal condition met!\n");
                int curr_id = curr.node_id;
                printf("\tPosition: %s\n", curr.pos.toString().c_str());
                while(curr_id != 0){
                    curr_id = pred_lookup[curr_id];
                    printf("\tPosition: %s\n", id_lookup[curr_id].toString().c_str());
                }
                break;
            }

            std::vector<Node> neighbors = get_neighbors(curr, map, movement, lookup, heuristic_type, last_id_assigned);
            if(neighbors.size() > 0){
                last_id_assigned = neighbors.back().node_id;
            }
            for(Node& nbr : neighbors){
                pred_lookup[nbr.node_id] = curr.node_id;
                queue.push(nbr);
            }
        }

        printf("Total nodes expanded: %d\n", num_expanded);

        return {};

    }

}