#pragma once

#include "shared.hpp"
#include "utils.hpp"
#include "pathfinding.hpp"
#include <queue>
#include <unordered_map>
#include <iostream>
#include <chrono>
#include <boost/functional/hash.hpp> // For boost::hash_value

double TOTAL_HEURISTIC_TIME = 0.0;
double TOTAL_TSP_SOLVER_TIME = 0.0;

int get_singleton_f_value(const std::vector<int>& agent_map_idxs, const std::vector<int>& agent_current_costs, int node_cost, const boost::dynamic_bitset<>& seen, const std::vector<Task>& tasks_left, const Lookup& lookup){
    int f_value = 0;
    for(int i = 0; i < seen.size(); i++){
        if(!seen[i]) {
            int closest_agent_f_value_to_see = INT_MAX;
            for(int j = 0; j < agent_map_idxs.size(); j++){
                int agent_map_idx = agent_map_idxs[j];
                closest_agent_f_value_to_see = std::min(closest_agent_f_value_to_see, lookup.min_dist_to_see[agent_map_idx][i] + agent_current_costs[j]);
            }

            f_value = std::max(f_value, closest_agent_f_value_to_see);
        }
    }
    for(const Task& task : tasks_left){
        int best_task_f_value = INT_MAX;
        // Find the closest agent and how long it would take to reach the task.
        for(int agent_map_idx : agent_map_idxs){
            best_task_f_value = std::min(best_task_f_value, lookup.apsp[agent_map_idx][task.map_idx]);
            best_task_f_value = std::max(best_task_f_value, task.min_time); // Ensure that we wait for the task release time.
        }
        f_value = std::max(f_value, best_task_f_value);
    }
    return std::max(node_cost, f_value);
}

// NOTE: Assumes that there is only one agent.
int get_mst_heuristic(const DisjointGraph& disjoint_graph){
    std::vector<std::vector<int>> adjacency_matrix = disjoint_graph.pivot_pivot_costs;
    for(int i = 0; i < disjoint_graph.pivots.size(); i++){
        adjacency_matrix[i].push_back(disjoint_graph.agent_pivot_costs[0][i]);
    }
    adjacency_matrix.push_back(disjoint_graph.agent_pivot_costs[0]);
    adjacency_matrix.back().push_back(0);

    // Calculate minimum spanning tree.
    std::priority_queue<std::tuple<int, int, int>> edge_set; // 
    int mst_heuristic = 0;
    std::vector<int> distances(adjacency_matrix.size(), INT_MAX);
    std::vector<bool> added(adjacency_matrix.size(), false);

    // printf("\nMST Calculation:\n");

    for(int i = 0; i < adjacency_matrix.size(); i++){
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
            distances[j] = std::min(distances[j], adjacency_matrix[next_node_to_add][j]);
        }

        added[next_node_to_add] = true;
    }

    // printf("MST Heuristic: %d\n", mst_heuristic);
    return mst_heuristic;
}

// NOTE: Assumes that there is only one agent.
std::tuple<int, std::vector<int>> get_tsp_heuristic(const DisjointGraph& disjoint_graph){
    auto heuristic_start = std::chrono::high_resolution_clock::now();

    // Create distance matrix. Add in a dummy node that connects to agent with cost 0 and all pivots with large cost (U).

    // // Max cost for any edge is map width * map height.
    // // Number of edges is number of pivots squared.
    // // Since U must be greater than sum of all edges, a safe value is width * height * number_of_pivots^2 + 1.
    // int U = los.size() * disjoint_graph.nodes.size() * disjoint_graph.nodes.size() + 1;

    // TSP path length is at most number of nodes * max cost per edge.
    int U = disjoint_graph.max_edge_cost * (disjoint_graph.pivot_pivot_costs.size() + 4); // +4 just to be safe.

    // The last node in distjoint_graph.edge_costs is the agent position.
    // We're adding a new dummy node at the end, so now the last row / col will be the dummy node and
    // the second to last row / col will be the agent node.
    std::vector<std::vector<int>> dist = disjoint_graph.pivot_pivot_costs;
    for(int i = 0; i < disjoint_graph.pivots.size(); i++){
        dist[i].push_back(disjoint_graph.agent_pivot_costs[0][i]);
    }
    dist.push_back(disjoint_graph.agent_pivot_costs[0]);
    dist.back().push_back(0);

    for(auto& row : dist){
        row.push_back(U);
    }
    dist.push_back(std::vector<int>(disjoint_graph.pivot_pivot_costs.size() + 2, U));

    // Dummy <-> Dummy cost is 0. Dummy <-> Agent cost is 0. Dummy <-> Pivot cost is U.
    dist[dist.size() - 1][dist.size() - 1] = 0;
    dist[dist.size() - 1][dist.size() - 2] = 0;
    dist[dist.size() - 2][dist.size() - 1] = 0;

    // printf("U: %d\n", U);
    // printf("TSP Distance Matrix:\n");
    // for(auto row : dist){
    //     for(int val : row){
    //         printf("%d ", val);
    //     }
    //     printf("\n");
    // }

    auto solver_start = std::chrono::high_resolution_clock::now();
    auto [ tsp_solution, tsp_path ] = pathfinding::solve_tsp(dist);

    // printf("TSP Solution: %d\n", tsp_solution);
    // printf("TSP Path: ");
    // for(int i : tsp_path){
    //     if(i == dist.size() - 1){
    //         printf("Dummy -> ");
    //     } else if(i == dist.size() - 2){
    //         printf("Agent -> ");
    //     } else {
    //         printf("Pivot %d -> ", i);
    //     }
    // }
    // printf("END\n");

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
        if(i == dist.size() - 2){
            tsp_pivots_path.push_back(-1); // Agent position
            continue;
        }
        tsp_pivots_path.push_back(disjoint_graph.pivots[i]);
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

int get_multi_tsp_f_value(const DisjointGraph& disjoint_graph, const std::vector<int>& agent_costs){
    auto heuristic_start = std::chrono::high_resolution_clock::now();

    std::vector<std::vector<int>> cost_map = disjoint_graph.pivot_pivot_costs;
    for(int i = 0; i < disjoint_graph.agent_pivot_costs.size(); i++){
        cost_map.push_back(disjoint_graph.agent_pivot_costs[i]);
    }

    TOTAL_MTSP_CALLS += 1;
    // printf("Calls: %d\n", TOTAL_MTSP_CALLS);
    int mtsp_solution = run_mtsp(agent_costs.size(), disjoint_graph.pivots.size(), disjoint_graph.min_task_times.size(), cost_map, agent_costs, disjoint_graph.min_task_times);
    // int mtsp_solution2 = run_mtsp2(agent_costs.size(), disjoint_graph.pivots.size(), disjoint_graph.min_task_times.size(), cost_map, agent_costs, disjoint_graph.min_task_times);
    // if(mtsp_solution != mtsp_solution2){
    //     printf("MTSP solutions don't match! %d != %d\n", mtsp_solution, mtsp_solution2);
    //     exit(1);
    // }
    return mtsp_solution;
}
