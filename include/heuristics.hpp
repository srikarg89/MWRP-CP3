#pragma once

#include "shared.hpp"
#include "utils.hpp"
#include "pathfinding.hpp"
#include <queue>
#include <unordered_map>
#include <iostream>
#include <chrono>
#include <boost/functional/hash.hpp> // For boost::hash_value

struct HeuristicInput {
    std::vector<AgentState> agents;
    int cost;
    boost::dynamic_bitset<> seen;
    std::vector<Task> tasks_left;
    int num_seen;
};

int get_singleton_f_value(const std::vector<AgentState>& agents, const Map& map, int node_cost, const boost::dynamic_bitset<>& seen, const std::vector<Task>& tasks_left, const Lookup& lookup){
    int f_value = 0;
    std::unordered_map<int, int> min_time_to_complete_task;
    for(const Task& task : tasks_left){
        int ttc = get_min_time_for_task_completion(agents, map, task, lookup, true);
        min_time_to_complete_task[task.id] = ttc;
        f_value = std::max(f_value, ttc);
    }
    for(int i = 0; i < seen.size(); i++){
        if(seen[i]) {
            continue;
        }
        int closest_agent_f_value_to_see = INT_MAX;
        for(const AgentState& agent : agents){
            if(agent.terminated){
                continue;
            }
            int agent_map_idx = map.get_map_idx(agent.pos);
            int agent_cost_before_moving = agent.cost;
            if(agent.waiting_idx != -1){
                // Agent is waiting at a task, so its "release time" from this task is effectively the time it'll take to complete that task.
                agent_cost_before_moving = std::max(agent_cost_before_moving, min_time_to_complete_task[agent.waiting_idx]);
            }
            closest_agent_f_value_to_see = std::min(closest_agent_f_value_to_see, lookup.min_dist_to_see[agent_map_idx][i] + agent_cost_before_moving);
        }

        f_value = std::max(f_value, closest_agent_f_value_to_see);
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

    return mst_heuristic;
}

std::pair<int, int> get_multi_tsp_f_and_focal_value(const DisjointGraph& disjoint_graph, const std::vector<int>& agent_costs, FocalMethod focal_method){
    auto heuristic_start = std::chrono::high_resolution_clock::now();

    std::vector<std::vector<int>> cost_map = disjoint_graph.pivot_pivot_costs;
    for(int i = 0; i < disjoint_graph.agent_pivot_costs.size(); i++){
        cost_map.push_back(disjoint_graph.agent_pivot_costs[i]);
    }

    // printf("Pivots:\n");
    // for(int i = 0; i < disjoint_graph.pivots.size(); i++){
    //     int p = disjoint_graph.pivots[i];
    //     printf("\t%d: %d\n", i, p);
    // }
    // for(int i = 0; i < disjoint_graph.num_required_visits.size(); i++){
    //     printf("Pivot %d requires %d visits\n", i, disjoint_graph.num_required_visits[i]);
    // }
    if(disjoint_graph.pivots.size() != disjoint_graph.num_required_visits.size()){
        printf("Pivots size %ld != num required visits size %ld\n", disjoint_graph.pivots.size(), disjoint_graph.num_required_visits.size());
        exit(1);
    }
    auto mtsp_solution = run_mtsp(agent_costs.size(), disjoint_graph.pivots.size(), cost_map, agent_costs, disjoint_graph.num_required_visits, focal_method);
    return mtsp_solution;
}
