#include "shared.hpp"
#include "watchman.hpp"
#include <queue>

namespace watchman {
    void add_los_to_seen(std::vector<bool>& seen, const std::vector<Position>& los, const Map& map){
        for(Position pos : los){
            seen[map.get_map_idx(pos)] = true;
        }
    }

    int get_heuristic(Position pos){
        return 1;
    }

    void get_neighbors(Node& node, const Map& map){
        for(Position pos : los){
            seen[map.get_map_idx(pos)] = true;
        }
    }

    // Inputs: Agent starting position, LOS type, map.
    // TODO: Add in radius for LOS.
    // Output: Optimal path.
    std::vector<Position> run_watchman(Position start, LOSType los, const Map& map){
        std::vector<std::vector<Position>> LOS_lookup;
        printf("Running watchman method!\n");
        for(int map_idx = 0; map_idx < map.x_size * map.y_size; map_idx++){
            Position pos = map.get_pos_from_map_idx(map_idx);
            if(map.check_obstacle(pos)){
                LOS_lookup.push_back({});
            } else {
                switch(los){
                    case FOUR_WAY_LOS:
                        LOS_lookup.push_back(four_way_LOS(pos, map));
                        break;
                    case EIGHT_WAY_LOS:
                        LOS_lookup.push_back(eight_way_LOS(pos, map));
                        break;
                    case BRES_LOS:
                        LOS_lookup.push_back(bresLOS(pos, map));
                        break;                        
                }
            }
            printf("Position: %s\n\t%s\n", pos.toString().c_str(), pos_array_to_string(LOS_lookup.back()).c_str());
        }

        // TODO: Precompute APSP? Just doin git lazily for now.

        // Initialize search data structures.
        std::priority_queue<Node, std::vector<Node>, std::greater<Node>> queue;

        // Setup the starting variable.
        std::vector<bool> start_seen(map.x_size * map.y_size, false);
        add_los_to_seen(start_seen, LOS_lookup[map.get_map_idx(start)], map);
        int start_heuristic = get_heuristic(start);
        queue.push(Node(start, start_seen, start_heuristic));

        while(!queue.empty()){
            Node curr = queue.top();
            queue.pop();

            // TODO: Expand curr.
        }

        return {};

    }

}