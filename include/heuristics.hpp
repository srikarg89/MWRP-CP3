#pragma once

#include "watchman.hpp"
#include "shared.hpp"
#include "watchman_utils.hpp"
#include "pathfinding.hpp"
#include <queue>
#include <unordered_map>
#include <iostream>
#include <chrono>
#include <boost/functional/hash.hpp> // For boost::hash_value

namespace watchman {
    double TOTAL_HEURISTIC_TIME = 0.0;
    double TOTAL_TSP_SOLVER_TIME = 0.0;

    int get_bfs_heuristic(){
        return 1;
    }

    int get_singleton_heuristic(int node_map_idx, const boost::dynamic_bitset<>& seen, const std::vector<std::vector<int>>& min_dist_to_see){
        int heuristic = 0;
        for(int i = 0; i < seen.size(); i++){
            if(!seen[i]) {
                heuristic = std::max(heuristic, min_dist_to_see[node_map_idx][i]);
            }
        }
        return heuristic;
    }

    int get_mst_heuristic(const DisjointGraph& disjoint_graph){
        // Calculate minimum spanning tree.
        std::priority_queue<std::tuple<int, int, int>> edge_set; // 
        int mst_heuristic = 0;
        std::vector<int> distances(disjoint_graph.nodes.size(), INT_MAX);
        std::vector<bool> added(disjoint_graph.nodes.size(), false);

        // printf("\nMST Calculation:\n");

        for(int i = 0; i < disjoint_graph.nodes.size(); i++){
            // Find the min cost node.
            int next_node_to_add = 0;
            int min_cost = INT_MAX;
            for(int j = 0; j < distances.size(); j++){
                if(added[j]){
                    continue;
                }
                if(distances[j] < min_cost){
                    next_node_to_add = j;
                    min_cost = distances[j];
                }
            }

            if(i > 0){
                mst_heuristic += min_cost;
            }

            for(int j = 0; j < distances.size(); j++){
                distances[j] = std::min(distances[j], disjoint_graph.edge_costs[next_node_to_add][j]);
            }

            added[next_node_to_add] = true;
        }

        // printf("MST Heuristic: %d\n", mst_heuristic);
        return mst_heuristic;
    }

    std::tuple<int, std::vector<int>> get_tsp_heuristic(const DisjointGraph& disjoint_graph){
        auto heuristic_start = std::chrono::high_resolution_clock::now();

        // Create distance matrix. Add in a dummy node that connects to agent with cost 0 and all pivots with large cost (U).

        // // Max cost for any edge is map width * map height.
        // // Number of edges is number of pivots squared.
        // // Since U must be greater than sum of all edges, a safe value is width * height * number_of_pivots^2 + 1.
        // int U = los.size() * disjoint_graph.nodes.size() * disjoint_graph.nodes.size() + 1;

        // TSP path length is at most number of nodes * max cost per edge.
        int U = disjoint_graph.max_edge_cost * (disjoint_graph.nodes.size() + 3); // +3 just to be safe.

        // The last node in distjoint_graph.edge_costs is the agent position.
        // We're adding a new dummy node at the end, so now the last row / col will be the dummy node and
        // the second to last row / col will be the agent node.
        std::vector<std::vector<int>> dist = disjoint_graph.edge_costs;
        for(auto& row : dist){
            row.push_back(U);
        }
        dist.push_back(std::vector<int>(disjoint_graph.nodes.size() + 1, U));

        // Dummy <-> Dummy cost is 0. Dummy <-> Agent cost is 0. Dummy <-> Pivot cost is U.
        dist[dist.size() - 1][dist.size() - 1] = 0;
        dist[dist.size() - 1][dist.size() - 2] = 0;
        dist[dist.size() - 2][dist.size() - 1] = 0;

        auto solver_start = std::chrono::high_resolution_clock::now();
        auto [ tsp_solution, tsp_path ] = pathfinding::solve_tsp(dist);

        // Reorder the path to remove the dummy node and start at the agent node.
        while(tsp_path[0] != dist.size() - 2){
            std::rotate(tsp_path.begin(), tsp_path.begin() + 1, tsp_path.end());
        }
        if(tsp_path[1] == dist.size() - 1){
            tsp_path.erase(tsp_path.begin() + 1);
        } else if(tsp_path.back() == dist.size() - 1){
            tsp_path.pop_back();
        } else {
            printf("WTF WHERE IS THE PATH\n");
            exit(0);
        }

        std::vector<int> tsp_pivots_path;
        for(int i : tsp_path){
            tsp_pivots_path.push_back(disjoint_graph.nodes[i]);
        }

        int tsp_heuristic = tsp_solution - U; // Subtract out the cost of the dummy -> pivot edge.

        auto end = std::chrono::high_resolution_clock::now();
        auto heuristic_seconds_taken = std::chrono::duration<double>(end - heuristic_start).count();
        auto tsp_solver_seconds_taken = std::chrono::duration<double>(end - solver_start).count();
        TOTAL_HEURISTIC_TIME += heuristic_seconds_taken;
        TOTAL_TSP_SOLVER_TIME += tsp_solver_seconds_taken;

        assert(tsp_heuristic >= 0);
        return std::make_tuple(tsp_heuristic, tsp_pivots_path);
    }
}