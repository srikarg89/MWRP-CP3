#pragma once

#include <unordered_set>
#include "shared.hpp"
#include "utils.hpp"

inline void write_node_to_file(std::ofstream& file, const Node& node, Lookup& lookup, const Map& map, int parent_id, HeuristicType heuristic_type){
    std::unordered_set<int> original_pivots;
    std::unordered_set<int> pivots;
    std::unordered_set<int> watchers;
    std::vector<int> agent_map_idxs;
    for(const AgentState& agent : node.agents){
        agent_map_idxs.push_back(map.get_map_idx(agent.pos));
    }

    if(heuristic_type == TSP || heuristic_type == MST || heuristic_type == MAX) {
        DisjointGraph disjoint_graph = compute_disjoint_graph(lookup, agent_map_idxs, node.seen, node.tasks_left);
        for(int pivot : disjoint_graph.pivots){
            original_pivots.insert(pivot);
        }
        prune_graph(disjoint_graph, lookup);
        for(int pivot : disjoint_graph.pivots){
            pivots.insert(pivot);
            for(Position watcher : lookup.los[pivot]){
                watchers.insert(map.get_map_idx(watcher));
            }
        }
    }

    std::string map_list = "";
    for(int i = 0; i < node.seen.size(); i++){
        if(pivots.find(i) != pivots.end()){
            map_list += "3"; // Pivot
        } else if(original_pivots.find(i) != original_pivots.end()){
            map_list += "5"; // Original Graph Pivot
        } else if(watchers.find(i) != watchers.end()){
            map_list += "4"; // Watcher
        }
        else if(node.seen[i]){
            map_list += "2"; // Seen
        } else {
            map_list += "0"; // Not seen
        }
    }
    // file << "Node ID, X, Y, Cost, Heuristic, F Value, Num Seen, Map Bitset\n"; // Header
    file << node.node_id << "," << parent_id << "," << node.agents.size() << "," << node.cost << "," << node.heuristic << "," << node.f_value << "," << node.num_seen << ",";
    for(AgentState agent : node.agents){
        file << agent.pos.x << "," << agent.pos.y << ",";
    }
    file << map_list << "\n";
}

inline void write_solution_to_file(std::ofstream& file, std::vector<std::vector<Position>> paths, const Map& map, const Lookup& lookup){
    boost::dynamic_bitset<> seen(map.x_size * map.y_size);
    int num_seen = 0;
    for(int map_idx = 0; map_idx < map.x_size * map.y_size; map_idx++){
        if(map.check_obstacle(map.get_pos_from_map_idx(map_idx))){
            seen[map_idx] = 1;
            num_seen += 1;
        }
    }
    std::vector<AgentState> start_agent_states;
    size_t max_length = 0;
    for(int i = 0; i < paths.size(); i++){
        max_length = std::max(max_length, paths[i].size());
    }

    for(auto& path : paths){
        while(path.size() < max_length){
            path.push_back(path.back());
        }
    }

    for(int t = 0; t < max_length; t++){
        std::vector<Position> curr_positions;
        for(int p = 0; p < paths.size(); p++){
            Position pos = paths[p][t];
            curr_positions.push_back(pos);
            num_seen += add_los_to_seen(seen, lookup.los[map.get_map_idx(pos)], map);
        }

        std::string map_list = "";
        for(int i = 0; i < seen.size(); i++){
            if(seen[i]){
                map_list += "2"; // Seen
            } else {
                map_list += "0"; // Not seen
            }
        }
        file << t << "," << curr_positions.size() << "," << num_seen << ",";
        for(Position pos : curr_positions){
            file << pos.x << "," << pos.y << ",";
        }
        file << map_list << "\n";
    }
}

inline void write_run_state_to_file(std::ofstream& file, int timestep, std::vector<std::vector<Position>> paths_to_go, const Map& map, std::vector<Position> agent_positions, boost::dynamic_bitset<> seen, std::vector<Position> known_incomplete_tasks, std::vector<Position> completed_tasks, std::vector<Position> unknown_tasks) {
    std::vector<int> agent_map_idxs = map.get_map_idxs(agent_positions);
    std::vector<int> known_incomplete_task_map_idxs = map.get_map_idxs(known_incomplete_tasks);
    std::vector<int> completed_task_map_idxs = map.get_map_idxs(completed_tasks);
    std::vector<int> unknown_task_map_idxs = map.get_map_idxs(unknown_tasks);

    // Create seen / agent state map list.
    std::string map_list = "";
    for(int i = 0; i < seen.size(); i++){
        if(std::find(agent_map_idxs.begin(), agent_map_idxs.end(), i) != agent_map_idxs.end()){
            map_list += "1"; // Agent
        } else if(seen[i]){
            map_list += "2"; // Seen
        } else {
            map_list += "0"; // Not seen
        }
    }

    std::string task_list = "";
    for(int i = 0; i < seen.size(); i++){
        if(std::find(known_incomplete_task_map_idxs.begin(), known_incomplete_task_map_idxs.end(), i) != known_incomplete_task_map_idxs.end()){
            task_list += "1"; // Known Incomplete Task
        } else if(std::find(completed_task_map_idxs.begin(), completed_task_map_idxs.end(), i) != completed_task_map_idxs.end()){
            task_list += "2"; // Completed Task
        } else if(std::find(unknown_task_map_idxs.begin(), unknown_task_map_idxs.end(), i) != unknown_task_map_idxs.end()){
            task_list += "3"; // Unknown Task
        } else {
            task_list += "0"; // Not a task
        }
    }

    // TODO: Add in paths to go.

    // file << "Timestep, Num Agents, Agent positions, Seen Bitset"; // Header
    file << timestep << "," << map_list << "," << task_list << "\n";
}
