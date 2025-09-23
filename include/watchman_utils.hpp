#pragma once

#include <vector>
#include <boost/dynamic_bitset.hpp>
#include "shared.hpp"
#include "pathfinding.hpp"

namespace watchman {

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

    // Returns the number of new squares marked as seen.
    int add_los_to_seen(boost::dynamic_bitset<>& seen, const std::vector<Position>& los, const Map& map){
        int count = 0;
        for(Position pos : los){
            int map_idx = map.get_map_idx(pos);
            if(!seen[map_idx]){
                count += 1;
            }
            seen[map_idx] = 1;
        }
        return count;
    }

    void precompute_lookup(Lookup& lookup, LOSType los, const Map& map, MovementType movement, HeuristicType heuristic_type){
        // Precompute the LOS Lookup and the All Pairs Shortest Path (APSP)
        printf("Running watchman method!\n");
        std::vector<int> sorted_los_order;
        for(int map_idx = 0; map_idx < map.x_size * map.y_size; map_idx++){
            Position pos = map.get_pos_from_map_idx(map_idx);
            if(map.check_obstacle(pos)){
                lookup.los.push_back({});
                std::vector<int> infinite_distances(map.x_size * map.y_size, INT_MAX);
                lookup.apsp.push_back(infinite_distances);
                lookup.apsp_paths.push_back(infinite_distances);
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

                auto [distances, preds] = pathfinding::get_bfs_distances_and_preds({pos}, movement, map);
                lookup.apsp.push_back(distances);
                lookup.apsp_paths.push_back(preds);
                sorted_los_order.push_back(map_idx);
            }
            // printf("Position: %s\n\t%s\n", pos.toString().c_str(), pos_array_to_string(lookup.los.back()).c_str());
        }

        printf("\n\n");

        std::sort(sorted_los_order.begin(), sorted_los_order.end(), [lookup](int a, int b){
            return lookup.los[a].size() < lookup.los[b].size();
        });
        lookup.sorted_los_order = sorted_los_order;

        // Precompute the Singleton heuristic helper lookup table.
        if(heuristic_type == HeuristicType::SINGLETON){
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
        }

        // Precompute the distance between pivots in G_DLC
        if(heuristic_type == HeuristicType::MST || heuristic_type == HeuristicType::TSP){
            for(int map_idx = 0; map_idx < map.x_size * map.y_size; map_idx++){
                std::vector<int> min_dists(map.x_size * map.y_size, INT_MAX);
                lookup.pivot_pivot_dists.push_back(min_dists);
            }

            for(int pivot_idx = 0; pivot_idx < map.x_size * map.y_size; pivot_idx++){
                if(map.check_obstacle(map.get_pos_from_map_idx(pivot_idx))){
                    std::vector<int> infinite_distances(map.x_size * map.y_size, INT_MAX);
                    lookup.pivot_cell_dists.push_back(infinite_distances);
                    lookup.pivot_pivot_dists.push_back(infinite_distances);
                    continue;
                }

                // For each pivot, we want to compute the distances from the pivot component to every other location.
                // Then, we can use that data to calculate the min dstance from the pivot component to any other pivot component.
                auto [pivot_cell_dists, _] = pathfinding::get_bfs_distances_and_preds(lookup.los[pivot_idx], movement, map);
                lookup.pivot_cell_dists.push_back(pivot_cell_dists);

                for(int cell_idx = 0; cell_idx < map.x_size * map.y_size; cell_idx++){
                    for(Position los_pos : lookup.los[cell_idx]){
                        int los_map_idx = map.get_map_idx(los_pos);
                        lookup.pivot_pivot_dists[pivot_idx][cell_idx] = std::min(lookup.pivot_pivot_dists[pivot_idx][cell_idx], pivot_cell_dists[los_map_idx]);
                    }
                }
            }
        }
    }

    DisjointGraph compute_disjoint_graph(const Lookup& lookup, int agent_map_idx, const boost::dynamic_bitset<>& seen){
        // Step 1: Get all the nodes: Agent position, pivots, watchers. We already processed the distances between pivot components, so don't need to add in the watchers.
        std::vector<int> pivots;

        for(int potential_pivot : lookup.sorted_los_order){
            // printf("Processing potential pivot: %d\n", potential_pivot);
            if(seen[potential_pivot]){
                // printf("\tSkipping cuz already seen\n");
                continue;
            }

            // We can test if a pivot is valid by checking the pivot <-> pivot distance. If this is 0, then the pivots share a watcher, and thus this pivot is invalid.
            // Technically still O(N^2) but now only checking against pivots instead of every cell, which should be much faster.
            bool valid = true;
            for(int existing_pivot : pivots){
                if(lookup.pivot_pivot_dists[existing_pivot][potential_pivot] == 0){
                    valid = false;
                    // printf("\tSkipping cuz intersects with existing pivot: %d\n", existing_pivot);
                    break;
                }
            }
            if(!valid){
                continue;
            }

            pivots.push_back(potential_pivot);
        }

        int max_edge_cost = 0;
        std::vector<std::vector<int>> edge_costs;
        std::vector<int> agent_costs;
        for(int p1 : pivots){
            // Add outgoing edges for each pivot.
            std::vector<int> pivot_outgoing_costs;
            for(int p2 : pivots){
                pivot_outgoing_costs.push_back(lookup.pivot_pivot_dists[p1][p2]);
                max_edge_cost = std::max(max_edge_cost, lookup.pivot_pivot_dists[p1][p2]);
            }
            pivot_outgoing_costs.push_back(lookup.pivot_cell_dists[p1][agent_map_idx]);
            agent_costs.push_back(lookup.pivot_cell_dists[p1][agent_map_idx]);
            edge_costs.push_back(pivot_outgoing_costs);

            max_edge_cost = std::max(max_edge_cost, lookup.pivot_cell_dists[p1][agent_map_idx]);
        }

        // Add agent position to the list of nodes.
        std::vector<int> nodes = pivots;
        nodes.push_back(agent_map_idx);
        agent_costs.push_back(0);

        // Add outgoing edges for the agent.
        edge_costs.push_back(agent_costs);

        // Nodes = pivots + agent position.
        return DisjointGraph {
            .nodes=nodes,
            .edge_costs=edge_costs,
            .max_edge_cost=max_edge_cost
        };
    }

    std::tuple<std::vector<Position>, std::vector<int>, std::vector<std::unordered_set<int>>> get_frontier_neighbors(const Lookup& lookup, const boost::dynamic_bitset<>& seen, const Map& map, MovementType movement){
        std::unordered_set<int> watchers;
        std::vector<int> pivots;
        std::vector<Position> neighbors;

        for(int potential_pivot : lookup.sorted_los_order){
            // printf("Processing potential pivot: %d\n", potential_pivot);
            if(seen[potential_pivot]){
                // printf("\tSkipping cuz already seen\n");
                continue;
            }

            // We can test if a pivot is valid by checking the pivot <-> pivot distance. If this is 0, then the pivots share a watcher, and thus this pivot is invalid.
            // Technically still O(N^2) but now only checking against pivots instead of every cell, which should be much faster.
            bool valid = true;
            for(int existing_pivot : pivots){
                if(lookup.pivot_pivot_dists[existing_pivot][potential_pivot] == 0){
                    valid = false;
                    // printf("\tSkipping cuz intersects with existing pivot: %d\n", existing_pivot);
                    break;
                }
            }
            if(!valid){
                continue;
            }

            pivots.push_back(potential_pivot);
            watchers.insert(potential_pivot);
            printf("Adding pivot: %s\n", map.get_pos_from_map_idx(potential_pivot).toString().c_str());
            std::unordered_set<int> component;
            for(Position los_pos : lookup.los[potential_pivot]){
                int los_map_idx = map.get_map_idx(los_pos);
                component.insert(los_map_idx);
                watchers.insert(los_map_idx);
            }

            for(Position los_pos : lookup.los[potential_pivot]){
                std::vector<Position> los_neighbors = map.get_neighbors(los_pos, movement);
                bool is_frontier = false;
                for(Position neighbor : los_neighbors){
                    int neighbor_map_idx = map.get_map_idx(neighbor);
                    if(component.find(neighbor_map_idx) == component.end()){
                        is_frontier = true;
                        break;
                    }
                }
                if(is_frontier){
                    printf("\tAdding frontier watcher: %s\n", los_pos.toString().c_str());
                    neighbors.push_back(los_pos);
                }
            }
        }

        // Add in white cells.
        std::vector<std::unordered_set<int>> white_components;
        for(int map_idx = 0; map_idx < map.x_size * map.y_size; map_idx++){
            if(seen[map_idx] || watchers.find(map_idx) != watchers.end()){
                continue;
            }

            int white_pivot = map_idx;
            printf("Adding white pivot: %s\n", map.get_pos_from_map_idx(white_pivot).toString().c_str());
            pivots.push_back(white_pivot);
            std::unordered_set<int> component;
            std::vector<Position> component_pos;
            for(Position los_pos : lookup.los[white_pivot]){
                int los_map_idx = map.get_map_idx(los_pos);
                if(seen[los_map_idx] || watchers.find(los_map_idx) != watchers.end()){
                    continue;
                }
                component.insert(los_map_idx);
                component_pos.push_back(los_pos);
                watchers.insert(los_map_idx);
            }
            white_components.push_back(component);

            for(Position los_pos : component_pos){
                int los_map_idx = map.get_map_idx(los_pos);
                if(los_map_idx == white_pivot){
                    continue;
                }
                std::vector<Position> los_neighbors = map.get_neighbors(los_pos, movement);
                bool is_frontier = false;
                for(Position neighbor : los_neighbors){
                    int neighbor_map_idx = map.get_map_idx(neighbor);
                    if(component.find(neighbor_map_idx) == component.end()){
                        is_frontier = true;
                        printf("\tAdding frontier neighbor: %s since its neighbor %s isn't in component.\n", los_pos.toString().c_str(), neighbor.toString().c_str());
                        break;
                    }
                }
                if(is_frontier){
                    neighbors.push_back(los_pos);
                }
            }
        }

        return std::make_tuple(neighbors, pivots, white_components);
    }
}

// TODO: LOS isn't reversible T_T.