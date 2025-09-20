#include "shared.hpp"
#include "watchman.hpp"
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

    int get_heuristic(Position pos){
        return 1;
    }

    // TODO: Fix the cost calculation if we do weighted edges instead of just saying "node.cost + 1".
    std::vector<Node> get_neighbors(Node& node, const Map& map, MovementType movement, const LOSLookup& los_lookup, int last_id_assigned){
        std::vector<Position> neighbors = map.get_neighbors(node.pos, movement);
        std::vector<Node> neighbor_nodes;
        for(Position nbr : neighbors){
            std::vector<bool> nbr_seen = node.seen;
            int new_squares_seen = add_los_to_seen(nbr_seen, los_lookup[map.get_map_idx(nbr)], map);
            int nbr_heuristic = get_heuristic(nbr);
            last_id_assigned += 1;
            neighbor_nodes.push_back(Node(last_id_assigned, nbr, nbr_seen, node.cost + 1, nbr_heuristic, node.num_seen + new_squares_seen));
        }
        return neighbor_nodes;
    }

    // Inputs: Agent starting position, LOS type, map.
    // TODO: Add in radius for LOS.
    // Output: Optimal path.
    std::vector<Position> run_watchman(Position start, LOSType los, const Map& map, MovementType movement){
        LOSLookup lookup;
        printf("Running watchman method!\n");
        int num_free_squares = 0;
        for(int map_idx = 0; map_idx < map.x_size * map.y_size; map_idx++){
            Position pos = map.get_pos_from_map_idx(map_idx);
            if(map.check_obstacle(pos)){
                lookup.push_back({});
            } else {
                num_free_squares += 1;
                switch(los){
                    case FOUR_WAY_LOS:
                        lookup.push_back(four_way_LOS(pos, map));
                        break;
                    case EIGHT_WAY_LOS:
                        lookup.push_back(eight_way_LOS(pos, map));
                        break;
                    case BRES_LOS:
                        lookup.push_back(bresLOS(pos, map));
                        break;                        
                }
            }
            printf("Position: %s\n\t%s\n", pos.toString().c_str(), pos_array_to_string(lookup.back()).c_str());
        }

        // TODO: Precompute APSP? Just doin git lazily for now.

        // Initialize search data structures.
        std::priority_queue<Node, std::vector<Node>, std::greater<Node>> queue;
        std::unordered_map<int, int> pred_lookup;
        std::unordered_map<int, Position> id_lookup;

        // Setup the starting variable.
        std::vector<bool> start_seen(map.x_size * map.y_size, false);
        int num_start_seen = add_los_to_seen(start_seen, lookup[map.get_map_idx(start)], map);
        int start_heuristic = get_heuristic(start);
        queue.push(Node(/* id = */ 0, start, start_seen, /* cost = */ 0, start_heuristic, num_start_seen));

        int num_expanded = 0;
        int last_id_assigned = 0;

        while(!queue.empty()){
            Node curr = queue.top();
            queue.pop();

            id_lookup[curr.node_id] = curr.pos;

            num_expanded += 1;
            printf("Expanding node %d. Loc: %s, cost: %d, heuristic: %d, num seen: %d\n", num_expanded, curr.pos.toString().c_str(), curr.cost, curr.heuristic, curr.num_seen);

            if(curr.num_seen == num_free_squares){
                printf("Goal condition met!\n");
                int curr_id = curr.node_id;
                printf("\tPosition: %s\n", curr.pos.toString().c_str());
                while(curr_id != 0){
                    curr_id = pred_lookup[curr_id];
                    printf("\tPosition: %s\n", id_lookup[curr_id].toString().c_str());
                }
                break;
            }

            std::vector<Node> neighbors = get_neighbors(curr, map, movement, lookup, last_id_assigned);
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