#pragma once

#include <vector>
#include <boost/dynamic_bitset.hpp>
#include "shared.hpp"
#include "pathfinding.hpp"
#include "los.hpp"

static const int MAX_PIVOTS = INT_MAX;

std::vector<std::tuple<Position, int>> get_extended_neighbors(const Map& map, const Position& pos, const boost::dynamic_bitset<>& seen, const Lookup& lookup){
    std::queue<std::tuple<Position, int>> queue; // (position, cost)
    std::unordered_set<int> visited;
    std::vector<std::tuple<Position, int>> extended_neighbors;

    queue.push(std::make_tuple(pos, 0));

    while(!queue.empty()){
        auto [curr_pos, curr_cost] = queue.front();
        queue.pop();

        int curr_map_idx = map.get_map_idx(curr_pos);
        if(visited.find(curr_map_idx) != visited.end()){
            continue;
        }
        visited.insert(curr_map_idx);

        std::vector<Position> neighbors = map.get_neighbors(curr_pos);
        for(Position neighbor : neighbors){
            int neighbor_map_idx = map.get_map_idx(neighbor);
            if(visited.find(neighbor_map_idx) != visited.end()){
                continue;
            }

            bool is_new = false;
            for(Position visible : lookup.los[neighbor_map_idx]){
                if(!seen[map.get_map_idx(visible)]){
                    is_new = true;
                    break;
                }
            }
            if(is_new){
                extended_neighbors.push_back(std::make_tuple(neighbor, curr_cost + 1));
            } else {
                queue.push(std::make_tuple(neighbor, curr_cost + 1));
            }
        }
    }
    return extended_neighbors;
}

void precompute_lookup(Lookup& lookup, const Map& map, HeuristicType heuristic_type, std::vector<Position> agent_starts){
    // Precompute the LOS Lookup and the All Pairs Shortest Path (APSP)
    printf("Precomputing lookup!\n");
    auto start_time = std::chrono::high_resolution_clock::now();
    for(int map_idx = 0; map_idx < map.num_squares; map_idx++){
        lookup.watchers.push_back({});
    }

    for(int map_idx = 0; map_idx < map.num_squares; map_idx++){
        Position pos = map.get_pos_from_map_idx(map_idx);
        if(map.check_obstacle(pos)){
            lookup.los.push_back({});
            std::vector<int> infinite_distances(map.num_squares, INT_MAX);
            lookup.apsp.push_back(infinite_distances);
            lookup.apsp_paths.push_back(infinite_distances);
        } else {
            switch(map.los_type){
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

            for(Position los_pos : lookup.los.back()){
                int los_map_idx = map.get_map_idx(los_pos);
                lookup.watchers[los_map_idx].push_back(pos);
            }

            auto [distances, preds] = pathfinding::get_bfs_distances_and_preds({pos}, map);
            lookup.apsp.push_back(distances);
            lookup.apsp_paths.push_back(preds);
        }
    }

    // OG sorted LOS method. Sort by which pivots have the least number of watchers.
    // Watchers are squares that can see the pivot.
    std::vector<int> sorted_pivot_order;
    for(int map_idx = 0; map_idx < map.num_squares; map_idx++){
        if(map.check_obstacle(map.get_pos_from_map_idx(map_idx))){
            continue;
        }
        sorted_pivot_order.push_back(map_idx);
    }
    std::sort(sorted_pivot_order.begin(), sorted_pivot_order.end(), [lookup](int a, int b){
        return lookup.watchers[a].size() < lookup.watchers[b].size();
    });

    auto end_time = std::chrono::high_resolution_clock::now();
    std::chrono::duration<double> duration = end_time - start_time;
    printf("LOS and APSP precomputation time: %.6f seconds\n", duration.count());

    // New sorted LOS method based on centrality. Might be better for multi-agent.
    // // TODO: Is this "start_seen" part necessary??
    // boost::dynamic_bitset<> start_seen(map.num_squares, 0);
    // for(Position agent_start : agent_starts){
        // Mark all squares visible from the agent start as seen.
        // for(Position los_pos : lookup.los[map.get_map_idx(agent_start)]){
            // start_seen[map.get_map_idx(los_pos)] = 1;
    //     }
    // }

    // std::vector<std::tuple<int, int>> sorted_order;
    // for(int i = 0; i < lookup.apsp.size(); i++){
    //     if(start_seen[i] || lookup.los[i].size() == 0){
    //         continue;
    //     }
    //     int centrality = 0;
    //     for(int j = 0; j < lookup.apsp.size(); j++){
    //         if(start_seen[j] || lookup.los[j].size() == 0){
    //             continue;
    //         }
    //         centrality += lookup.apsp[i][j];
    //     }
    //     sorted_order.push_back(std::make_tuple(centrality, i));
    // }
    // std::sort(sorted_order.begin(), sorted_order.end(), std::greater<>());
    // std::vector<int> sorted_pivot_order;
    // for(const auto& [centrality, idx] : sorted_order){
    //     sorted_pivot_order.push_back(idx);
    // }

    lookup.sorted_pivot_order = sorted_pivot_order;

    // Precompute the Singleton heuristic helper lookup table.
    if(heuristic_type == HeuristicType::SINGLETON || heuristic_type == HeuristicType::MAX || heuristic_type == HeuristicType::LAZY){
        for(int map_idx = 0; map_idx < map.num_squares; map_idx++){
            std::vector<int> min_dists(map.num_squares, INT_MAX);
            lookup.min_dist_to_see.push_back(min_dists);
        }

        // For each target position.
        for(int g_map_idx = 0; g_map_idx < map.num_squares; g_map_idx++){
            // For each source position.
            for(int s_map_idx = 0; s_map_idx < map.num_squares; s_map_idx++){
                // Loop through each watcher.
                for(Position watcher_pos : lookup.watchers[g_map_idx]){
                    int watcher_idx = map.get_map_idx(watcher_pos);

                    // Calculate min dist to get to any watcher.
                    lookup.min_dist_to_see[s_map_idx][g_map_idx] = std::min(lookup.min_dist_to_see[s_map_idx][g_map_idx], lookup.apsp[s_map_idx][watcher_idx]);
                }
            }
        }
    }

    end_time = std::chrono::high_resolution_clock::now();
    duration = end_time - start_time;
    printf("Singleton precomputation time: %.6f seconds\n", duration.count());

    // Precompute the distance between pivots in G_DLC
    if(heuristic_type == HeuristicType::MST || heuristic_type == HeuristicType::TSP || heuristic_type == HeuristicType::MAX || heuristic_type == HeuristicType::LAZY){
        for(int map_idx = 0; map_idx < map.num_squares; map_idx++){
            std::vector<int> min_dists(map.num_squares, INT_MAX);
            lookup.pivot_pivot_dists.push_back(min_dists);
        }

        // Loop through each pivot and compute distances from pivot's watchers to the watchers of any other pivot and to any other cell.
        for(int pivot_idx = 0; pivot_idx < map.num_squares; pivot_idx++){
            if(map.check_obstacle(map.get_pos_from_map_idx(pivot_idx))){
                std::vector<int> infinite_distances(map.num_squares, INT_MAX);
                lookup.pivot_cell_dists.push_back(infinite_distances);
                lookup.pivot_pivot_dists.push_back(infinite_distances);
                continue;
            }

            // For each pivot, we want to compute the distances from the pivot component to every other location.
            // Then, we can use that data to calculate the min dstance from the pivot component to any other pivot component.
            auto [pivot_cell_dists, _] = pathfinding::get_bfs_distances_and_preds(lookup.watchers[pivot_idx], map);
            lookup.pivot_cell_dists.push_back(pivot_cell_dists);

            for(int cell_idx = 0; cell_idx < map.num_squares; cell_idx++){
                for(Position watcher_pos : lookup.watchers[cell_idx]){
                    int watcher_idx = map.get_map_idx(watcher_pos);
                    lookup.pivot_pivot_dists[pivot_idx][cell_idx] = std::min(lookup.pivot_pivot_dists[pivot_idx][cell_idx], pivot_cell_dists[watcher_idx]);
                }
            }
        }
    }
}

DisjointGraph compute_disjoint_graph(const Lookup& lookup, std::vector<int> agent_map_idxs, const boost::dynamic_bitset<>& seen, const std::vector<int>& tasks_left){
    // Step 1: Get all the nodes: Agent position, pivots, watchers. We already processed the distances between pivot components, so don't need to add in the watchers.
    std::vector<int> pivots;

    for(int potential_pivot : lookup.sorted_pivot_order){
    // for(auto [_, potential_pivot] : sorted_order){
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

        // Check if one of the tasks is already a watcher for this pivot.
        for(int task : tasks_left){
            if(lookup.pivot_cell_dists[potential_pivot][task] == 0){
                valid = false;
                break;
            }
        }

        if(!valid){
            continue;
        }

        // printf("\tUsing pivot: %d\n", potential_pivot);
        pivots.push_back(potential_pivot);
    }

    int num_exploration_pivots = pivots.size();
    for(int task : tasks_left){
        pivots.push_back(task);
    }

    int max_edge_cost = 0;
    std::vector<std::vector<int>> pivot_pivot_costs;
    std::vector<std::vector<int>> agent_pivot_costs(agent_map_idxs.size(), std::vector<int>());
    for(int i = 0; i < pivots.size(); i++){
        int p1 = pivots[i];
        // Add outgoing edges for each pivot.
        std::vector<int> pivot_outgoing_costs;
        for(int j = 0; j < pivots.size(); j++){
            int p2 = pivots[j];
            int cost = 0;
            if(p1 < num_exploration_pivots && p2 < num_exploration_pivots){ // P1 is exploration, P2 is exploration.
                cost = lookup.pivot_pivot_dists[p1][p2];
            } else if(p1 < num_exploration_pivots && p2 >= num_exploration_pivots) { // P1 is exploration, P2 is task.
                cost = lookup.pivot_cell_dists[p1][p2];
            } else if(p1 >= num_exploration_pivots && p2 < num_exploration_pivots) { // P1 is task, P2 is exploration.
                cost = lookup.pivot_cell_dists[p2][p1];
            } else if(p1 >= num_exploration_pivots && p2 >= num_exploration_pivots) { // P1 is task, P2 is task.
                cost = lookup.apsp[p1][p2];
            } else {
                printf("ERROR IN PIVOT COST COMPUTATION ???\n");
                exit(1);
            }

            pivot_outgoing_costs.push_back(cost);
            max_edge_cost = std::max(max_edge_cost, cost);
        }
        pivot_pivot_costs.push_back(pivot_outgoing_costs);

        for(int i = 0; i < agent_map_idxs.size(); i++){
            int cost = 0;
            if(p1 < num_exploration_pivots) {
                cost = lookup.pivot_cell_dists[p1][agent_map_idxs[i]];
            } else {
                cost = lookup.apsp[p1][agent_map_idxs[i]];
            }
            agent_pivot_costs[i].push_back(cost);
            max_edge_cost = std::max(max_edge_cost, cost);
        }
    }

    return DisjointGraph {
        .pivots=pivots,
        .pivot_pivot_costs=pivot_pivot_costs,
        .agent_pivot_costs=agent_pivot_costs,
        .max_edge_cost=max_edge_cost
    };
}

void prune_graph(DisjointGraph& graph, const Lookup& lookup){
    while(true){
        int shortcut_pivot = -1;
        int biggest_shortcut = 0;

        for(int i = 0; i < graph.pivots.size(); i++){
            // BFS out to find distance to every other pivot.
            std::vector<int> distances(graph.pivots.size(), INT_MAX);
            std::vector<int> pred(graph.pivots.size(), -1);
            distances[i] = 0;
            std::priority_queue<std::tuple<int, int>, std::vector<std::tuple<int, int>>, std::greater<>> queue; // cost, node idx
            queue.push(std::make_tuple(0, i));
            while(!queue.empty()){
                auto [curr_cost, curr_idx] = queue.top();
                queue.pop();

                if(distances[curr_idx] < curr_cost){
                    continue;
                }

                distances[curr_idx] = curr_cost;

                for(int neighbor_idx = 0; neighbor_idx < graph.pivots.size(); neighbor_idx++){
                    if(neighbor_idx == curr_idx){
                        continue;
                    }
                    int edge_cost = graph.pivot_pivot_costs[curr_idx][neighbor_idx];
                    if(edge_cost == INT_MAX){
                        continue;
                    }
                    int new_cost = curr_cost + edge_cost;
                    if(new_cost < distances[neighbor_idx]){
                        distances[neighbor_idx] = new_cost;
                        pred[neighbor_idx] = curr_idx;
                        queue.push(std::make_tuple(new_cost, neighbor_idx));
                    }
                }
            }

            for(int j = 0; j < graph.pivots.size(); j++){
                if(i == j || pred[j] == -1){
                    continue;
                }
                if(pred[j] != i && pred[pred[j]] == i){
                    int shortcut = graph.pivot_pivot_costs[i][j] - distances[j];
                    if(shortcut > biggest_shortcut) {
                        biggest_shortcut = shortcut;
                        shortcut_pivot = pred[j];
                    }
                }
            }
        }

        // No pivot provides a shortcut, so we're done pruning.
        if(biggest_shortcut <= 0){
            break;
        }

        graph.pivots.erase(graph.pivots.begin() + shortcut_pivot);
        graph.pivot_pivot_costs.erase(graph.pivot_pivot_costs.begin() + shortcut_pivot);
        for(auto& row : graph.pivot_pivot_costs){
            row.erase(row.begin() + shortcut_pivot);
        }
        for(auto& row : graph.agent_pivot_costs){
            row.erase(row.begin() + shortcut_pivot);
        }
    }

    // Prune pivots to be under the max allowed using farness centrality.
    // NOTE: Tried using MAX instead of SUM for farness, but MAX performed much worse.
    while(graph.pivots.size() > MAX_PIVOTS){
        int worst_pivot = -1;
        int worst_farness = INT_MAX;

        for(int i = 0; i < graph.pivots.size(); i++){
            int farness = 0;
            for(int j = 0; j < graph.pivots.size(); j++){
                if(i == j){
                    continue;
                }
                farness += graph.pivot_pivot_costs[i][j];
            }
            for(int j = 0; j < graph.agent_pivot_costs.size(); j++){
                farness += graph.agent_pivot_costs[j][i];
            }

            if(farness < worst_farness){
                worst_farness = farness;
                worst_pivot = i;
            }
        }

        graph.pivots.erase(graph.pivots.begin() + worst_pivot);
        graph.pivot_pivot_costs.erase(graph.pivot_pivot_costs.begin() + worst_pivot);
        for(auto& row : graph.pivot_pivot_costs){
            row.erase(row.begin() + worst_pivot);
        }
        for(auto& row : graph.agent_pivot_costs){
            row.erase(row.begin() + worst_pivot);
        }
    }
}
