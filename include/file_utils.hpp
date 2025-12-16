#pragma once

#include <unordered_set>
#include "shared.hpp"
#include "utils.hpp"

inline void write_node_to_file(std::ofstream& file, const Node& node, const Lookup& lookup, const Map& map, int parent_id, HeuristicType heuristic_type){
    std::unordered_set<int> pivots;
    std::unordered_set<int> watchers;
    std::vector<AgentState> non_terminated_agents;
    for(const AgentState& agent : node.agents){
        if(!agent.terminated){
            non_terminated_agents.push_back(agent);
        }
    }

    if(heuristic_type == TSP || heuristic_type == MAX) {
        DisjointGraph disjoint_graph = compute_disjoint_graph(map, non_terminated_agents, node.seen, lookup, INT_MAX);
        prune_graph(disjoint_graph, lookup);
        for(int i = 0; i < disjoint_graph.pivots.size(); i++){
            int pivot = disjoint_graph.pivots[i];
            pivots.insert(pivot);
            for(Position watcher : lookup.watchers[pivot]){
                watchers.insert(map.get_map_idx(watcher));
            }
        }
    }

    std::string map_list = "";
    for(int i = 0; i < node.seen.size(); i++){
        if(pivots.find(i) != pivots.end()){
            map_list += "3"; // Exploration Pivot
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

inline std::string get_map_state(const Lookup& lookup, const Map& map, boost::dynamic_bitset<> seen, const std::vector<Position>& agents){
    std::string map_list = "";
    std::unordered_set<int> agent_positions;
    for(const Position& pos : agents){
        agent_positions.insert(map.get_map_idx(pos));
    }

    auto cd_strictly_easier = calculate_square_dominance(map, lookup.watchers, lookup.watchers_set);

    for(int i = 0; i < map.num_squares; i++){
        if(map.check_obstacle(map.get_pos_from_map_idx(i))){
            map_list += "5"; // Obstacle
        } else if(agent_positions.find(i) != agent_positions.end()){
            map_list += "1"; // Agent
        } else if(seen[i]){
            map_list += "4"; // Seen
        } else if(cd_strictly_easier[i]){
            map_list += "3"; // Path-dominance Strictly easier
        } else if(lookup.strictly_easier[i]){
            map_list += "2"; // Path-dominance Strictly easier
        } else {
            map_list += "0"; // Not seen
        }
    }

    return map_list;
}

inline void write_run_state_to_file(std::ofstream& file, int timestep, std::vector<std::vector<Position>> paths_to_go, const Map& map, std::vector<Position> agent_positions, boost::dynamic_bitset<> seen) {
    std::vector<int> agent_map_idxs = map.get_map_idxs(agent_positions);

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

    // file << "Timestep, Num Agents, Agent positions, Seen Bitset"; // Header
    file << timestep << "," << map_list << ",";
    for(auto& path : paths_to_go){
        for(Position pos : path){
            file << map.get_map_idx(pos) << "_";
        }
        file << ",";
    }
    file << "\n";
}
