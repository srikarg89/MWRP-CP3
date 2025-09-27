#include "watchman.hpp"
#include "shared.hpp"
#include "watchman_utils.hpp"
#include "pathfinding.hpp"
#include "heuristics.hpp"
#include <queue>
#include <unordered_map>
#include <iostream>
#include <chrono>
#include <boost/functional/hash.hpp> // For boost::hash_value

int NUM_SKIPPED = 0;
int MAX_EXISTING_NODES_SIZE = 0;

namespace watchman {
    void prune_graph(DisjointGraph& graph, const Lookup& lookup){
        while(true){
            int shortcut_pivot = -1;
            int biggest_shortcut = 0;

            for(int i = 0; i < graph.nodes.size() - 1; i++){
                // BFS out to find distance to every other pivot.
                std::vector<int> distances(graph.nodes.size(), INT_MAX);
                std::vector<int> pred(graph.nodes.size(), -1);
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

                    for(int neighbor_idx = 0; neighbor_idx < graph.nodes.size(); neighbor_idx++){
                        if(neighbor_idx == curr_idx){
                            continue;
                        }
                        int edge_cost = graph.edge_costs[curr_idx][neighbor_idx];
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

                for(int j = 0; j < graph.nodes.size(); j++){
                    if(i == j || pred[j] == -1 || pred[j] == graph.nodes.size() - 1){
                        continue;
                    }
                    if(pred[j] != i && pred[pred[j]] == i){
                        int shortcut = graph.edge_costs[i][j] - distances[j];
                        if(shortcut > biggest_shortcut) {
                            biggest_shortcut = shortcut;
                            shortcut_pivot = pred[j];
                        }
                    }
                }
            }

            // No pivot provides a shortcut, so we're done pruning.
            if(shortcut_pivot == -1){
                break;
            }

            graph.nodes.erase(graph.nodes.begin() + shortcut_pivot);
            graph.edge_costs.erase(graph.edge_costs.begin() + shortcut_pivot);
            for(auto& row : graph.edge_costs){
                row.erase(row.begin() + shortcut_pivot);
            }
        }
    }


    int get_heuristic(HeuristicType heuristic_type, const Map& map, std::vector<AgentState> agent_states, const boost::dynamic_bitset<>& seen, const Lookup& lookup){
        std::vector<int> non_terminated_agent_map_idxs;
        for(const auto& agent : agent_states){
            if(!agent.terminated){
                non_terminated_agent_map_idxs.push_back(map.get_map_idx(agent.pos));
            }
        }
        switch(heuristic_type){
            case BFS:
                return get_bfs_heuristic();
            case SINGLETON:
                return get_singleton_heuristic(non_terminated_agent_map_idxs, seen, lookup.min_dist_to_see);
            case MST: {
                if(agent_states.size() != 1){
                    printf("MST heuristic only supports single-agent watchman.\n");
                    assert(false);
                    exit(1);
                }
                DisjointGraph disjoint_graph_mst = compute_disjoint_graph(lookup, map.get_map_idx(agent_states[0].pos), seen);

                // If there's no pivots, just return the singleton heuristic.
                if(disjoint_graph_mst.nodes.size() <= 1){
                    return get_singleton_heuristic(non_terminated_agent_map_idxs, seen, lookup.min_dist_to_see);
                }

                prune_graph(disjoint_graph_mst, lookup);

                return get_mst_heuristic(disjoint_graph_mst);
            }
            case TSP: {
                if(agent_states.size() != 1){
                    printf("TSP heuristic only supports single-agent watchman.\n");
                    assert(false);
                    exit(1);
                }

                // Calculate the disjoint graph.
                DisjointGraph disjoint_graph_tsp = compute_disjoint_graph(lookup, map.get_map_idx(agent_states[0].pos), seen);

                // If there's no pivots, just return the singleton heuristic.
                if(disjoint_graph_tsp.nodes.size() <= 1){
                    return get_singleton_heuristic(non_terminated_agent_map_idxs, seen, lookup.min_dist_to_see);
                }

                prune_graph(disjoint_graph_tsp, lookup);
                auto [ tsp_heuristic, tsp_path ] = get_tsp_heuristic(disjoint_graph_tsp);
                return tsp_heuristic;
            }
            default:
                printf("UNKNOWN HEURISTIC TYPE ???\n");
                assert(false);
                exit(1);
        }
        assert(false);
        return -1;
    }

    std::vector<std::vector<AgentState>> get_possible_moves(const Map& map, const std::vector<AgentState>& agents, MovementType movement, const boost::dynamic_bitset<>& seen, const Lookup& lookup, bool expanding_borders){
        std::vector<std::vector<AgentState>> options; 
        for(AgentState agent : agents){
            if(agent.terminated){
                // If any agent is terminated, we don't generate any neighbors.
                options.push_back({agent});
                continue;
            }
            std::vector<AgentState> agent_options;

            // Expanding borders implementation.
            if(expanding_borders){
                std::vector<std::tuple<Position, int>> nbrs_with_added_cost = get_extended_neighbors(map, agent.pos, movement, seen, lookup);
                for(auto [nbr, added_cost] : nbrs_with_added_cost){
                    agent_options.push_back(AgentState(nbr, false, agent.cost + added_cost));
                }
            }
            // Generic one-step implementation
            else {
                std::vector<Position> nbrs = map.get_neighbors(agent.pos, movement);
                for(Position nbr : nbrs){
                    agent_options.push_back(AgentState(nbr, false, agent.cost + 1));
                }
            }

            agent_options.push_back(AgentState(agent.pos, true, agent.cost)); // Option to terminate.
            options.push_back(agent_options);
        }
        // Now, take the cartesian product of options. Don't include states in which all of the agents terminate.
        std::vector<std::vector<AgentState>> all_moves;
        std::function<void(int, std::vector<AgentState>, bool)> backtrack = [&](int idx, std::vector<AgentState> current, bool has_non_terminated_agent){
            if(idx == options.size()){
                if(has_non_terminated_agent){
                    all_moves.push_back(current);
                }
                return;
            }
            for(AgentState option : options[idx]){
                current.push_back(option);
                backtrack(idx + 1, current, has_non_terminated_agent || !option.terminated);
                current.pop_back();
            }
        };
        backtrack(0, {}, false);
        return all_moves;
    }

    bool is_subset(Position pos, const boost::dynamic_bitset<>& seen, int num_seen, const Node& node){
        if(node.num_seen < num_seen){
            return false;
        }
        return (pos.x == node.agents[0].pos.x) && (pos.y == node.agents[0].pos.y) && ((seen & node.seen) == seen);
    }

    std::string bitset_to_string(const boost::dynamic_bitset<>& seen){
        std::string str = "[";
        for(int i = 0; i < seen.size(); i++){
            str += seen[i] ? "1" : "0";
        }
        str += "]";
        return str;
    }

    std::vector<Node> get_neighbors(Node& node, const Map& map, MovementType movement, const Lookup& lookup, SolverConfig solver_config, int last_id_assigned, std::unordered_map<std::tuple<std::string, size_t>, int, boost::hash<std::tuple<std::string, size_t>>>& generated_costs){
        std::vector<Node> neighbor_nodes;
        auto neighbors = get_possible_moves(map, node.agents, movement, node.seen, lookup, solver_config.expanding_borders);
        for(const auto& nbr : neighbors){
            boost::dynamic_bitset<> nbr_seen = node.seen;
            // For each neighbor, loop through path to neighbor.
            int new_squares_seen = 0;
            int nbr_cost = 0;
            for(const AgentState& agent : nbr){
                new_squares_seen += add_los_to_seen(nbr_seen, lookup.los[map.get_map_idx(agent.pos)], map);
                nbr_cost += agent.cost;
            }

            int nbr_num_seen = node.num_seen + new_squares_seen;

            std::tuple<std::string, size_t> nbr_key = std::make_tuple(agent_states_to_string(nbr), boost::hash_value(nbr_seen));
            if(generated_costs.find(nbr_key) != generated_costs.end()){
                if(nbr_cost >= generated_costs[nbr_key]){
                    // We've already generated a cheaper version of this node.
                    NUM_SKIPPED += 1;
                    continue;
                }
            }
            generated_costs[nbr_key] = nbr_cost;

            // Calculate heuristic.
            int nbr_heuristic = get_heuristic(solver_config.heuristic_type, map, nbr, nbr_seen, lookup);
            last_id_assigned += 1;
            neighbor_nodes.push_back(Node(last_id_assigned, nbr, nbr_seen, nbr_cost, nbr_heuristic, nbr_num_seen));
        }
        return neighbor_nodes;
    }

    void write_node_to_file(std::ofstream& file, const Node& node, Lookup& lookup, const Map& map, int parent_id, HeuristicType heuristic_type){
        std::unordered_set<int> original_pivots;
        std::unordered_set<int> pivots;
        std::unordered_set<int> watchers;
        std::unordered_set<int> agents;
        for(const AgentState& agent : node.agents){
            agents.insert(map.get_map_idx(agent.pos));
        }

        if(heuristic_type == TSP || heuristic_type == MST) {
            DisjointGraph disjoint_graph = compute_disjoint_graph(lookup, map.get_map_idx(node.agents[0].pos), node.seen);
            for(int pivot : disjoint_graph.nodes){
                if(agents.find(pivot) != agents.end()){
                    continue;
                }
                original_pivots.insert(pivot);
            }
            prune_graph(disjoint_graph, lookup);
            for(int pivot : disjoint_graph.nodes){
                if(agents.find(pivot) != agents.end()){
                    continue;
                }
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
        file << node.node_id << "," << parent_id << "," << node.agents.size() << ", " << node.cost << "," << node.heuristic << "," << node.f_value << "," << node.num_seen << ",";
        for(AgentState agent : node.agents){
            file << agent.pos.x << "," << agent.pos.y << ",";
        }
        file << map_list << "\n";}

    // Inputs: Agent starting positions, LOS type, map.
    // Output: Optimal path.
    std::vector<std::vector<Position>> run_watchman(std::vector<Position> starts, const ScenarioConfig& scenario_config, const SolverConfig& solver_config){
        Lookup lookup;
        precompute_lookup(lookup, scenario_config, solver_config.heuristic_type, starts);

        // Create initial seen bitset.
        boost::dynamic_bitset<> start_seen(scenario_config.map.x_size * scenario_config.map.y_size);
        int num_start_seen = 0;
        for(int map_idx = 0; map_idx < scenario_config.map.x_size * scenario_config.map.y_size; map_idx++){
            if(scenario_config.map.check_obstacle(scenario_config.map.get_pos_from_map_idx(map_idx))){
                start_seen[map_idx] = 1;
                num_start_seen += 1;
            }
        }
        std::vector<AgentState> start_agent_states;
        for(Position start : starts){
            num_start_seen += add_los_to_seen(start_seen, lookup.los[scenario_config.map.get_map_idx(start)], scenario_config.map);
            start_agent_states.push_back(AgentState(start, false, 0));
        }

        // Initialize search data structures.
        std::priority_queue<Node, std::vector<Node>, std::greater<Node>> queue;
        std::unordered_map<int, int> pred_lookup;
        std::unordered_map<int, std::vector<AgentState>> id_lookup;
        std::unordered_set<std::tuple<std::string, size_t>, boost::hash<std::tuple<std::string, size_t>>> visited_nodes; // (map_idx, seen bitset hash)
        std::unordered_map<std::tuple<std::string, size_t>, int, boost::hash<std::tuple<std::string, size_t>>> generated_costs; // (map_idx, seen bitset hash)

        std::ofstream debug_file;
        debug_file.open("watchman_debug.csv");
        debug_file << "Node ID, X, Y, Cost, Heuristic, F Value, Num Seen, Seen Bitset\n"; // Header

        // Setup the starting variable. Mark all obstacles as seen.
        int num_obstacles = num_start_seen;
        int num_free = scenario_config.map.x_size * scenario_config.map.y_size - num_obstacles;


        int start_heuristic = get_heuristic(solver_config.heuristic_type, scenario_config.map, start_agent_states, start_seen, lookup);
        printf("Start heuristic: %d\n", start_heuristic);
        queue.push(Node(/* id = */ 0, start_agent_states, start_seen, /* cost = */ 0, start_heuristic, num_start_seen));

        std::vector<Node> expanded_nodes;

        int num_expanded = 0;
        int num_generated = 0;
        int last_id_assigned = 0;
        int max_new_squares_seen = 0;
        int num_skipped = 0;
        int num_skipped_dom = 0;

        auto start_time = std::chrono::high_resolution_clock::now();

        std::vector<std::vector<Position>> path;

        while(!queue.empty()){
            Node curr = queue.top();
            queue.pop();

            size_t seen_hash = boost::hash_value(curr.seen);
            std::tuple<std::string, size_t> visited_key = std::make_tuple(agent_states_to_string(curr.agents), seen_hash);
            if(visited_nodes.find(visited_key) != visited_nodes.end()){
                // Already visited this node.
                num_skipped += 1;
                continue;
            }

            expanded_nodes.push_back(curr);

            visited_nodes.insert(visited_key);
            id_lookup[curr.node_id] = curr.agents;

            write_node_to_file(debug_file, curr, lookup, scenario_config.map, pred_lookup[curr.node_id], solver_config.heuristic_type);

            // debug_file.close(); exit(0);

            max_new_squares_seen = std::max(max_new_squares_seen, curr.num_seen - num_obstacles);
            num_expanded += 1;
            if(num_expanded % 1000 == 0){
            // if(num_expanded % 1 == 0){
                printf("Expanded %d nodes. Loc: %s, cost: %d, heuristic: %d, num new seen: %d / %d, max new squares seen: %d\n", num_expanded, agent_states_to_print_string(curr.agents).c_str(), curr.cost, curr.heuristic, (curr.num_seen - num_obstacles), num_free, max_new_squares_seen);
                printf("\tF value: %d. Next node's f value: %d\n", curr.f_value, (queue.empty() ? -1 : queue.top().f_value));
            }
            // printf("Expanding node %d. Node ID: %d, Loc: %s, cost: %d, heuristic: %d, num seen: %d\n", num_expanded, curr.node_id, agent_states_to_print_string(curr.agents).c_str(), curr.cost, curr.heuristic, curr.num_seen);

            if(curr.num_seen == scenario_config.map.x_size * scenario_config.map.y_size){
                printf("Goal condition met!\n");
                auto end_time = std::chrono::high_resolution_clock::now();
                auto seconds_taken = std::chrono::duration<double>(end_time - start_time).count();
                printf("Search time taken: %.3f seconds\n", seconds_taken);
                printf("Solution cost: %d\n", curr.cost);

                path.push_back(agent_states_to_positions(curr.agents));
                int curr_id = curr.node_id;
                while(curr_id != 0){
                    curr_id = pred_lookup[curr_id];
                    path.push_back(agent_states_to_positions(id_lookup[curr_id]));
                }
                std::reverse(path.begin(), path.end());
                break;;
            }

            std::vector<Node> neighbors = get_neighbors(curr, scenario_config.map, scenario_config.movement_type, lookup, solver_config, last_id_assigned, generated_costs);
            num_generated += neighbors.size();
            if(neighbors.size() > 0){
                last_id_assigned = neighbors.back().node_id;
            }
            for(Node& nbr : neighbors){
                // printf("\tGenerated neighbor. Node ID: %d, Loc: %s, cost: %d, heuristic: %d, num seen: %d\n", nbr.node_id, nbr.pos.toString().c_str(), nbr.cost, nbr.heuristic, nbr.num_seen);
                pred_lookup[nbr.node_id] = curr.node_id;
                queue.push(nbr);
            }
        }

        printf("Total nodes expanded: %d\n", num_expanded);
        printf("Total expansions skipped: %d\n", num_skipped);
        printf("Total expansions skipped by domination check: %d\n", num_skipped_dom);
        printf("Total generations skipped: %d\n", NUM_SKIPPED);
        printf("Total nodes generated: %d\n", num_generated);
        printf("Total heuristic time: %.3f seconds\n", TOTAL_HEURISTIC_TIME);
        printf("Total TSP solver time: %.3f seconds\n", TOTAL_TSP_SOLVER_TIME);
        printf("Total TSP brute force: %.6f seconds\n", TOTAL_TSP_BRUTE_FORCE_TIME);
        printf("Total TSP concorde time: %.6f seconds\n", TOTAL_TSP_CONCORDE_TIME);
        printf("Total brute force calls: %d\n", TOTAL_TSP_BRUTE_FORCE_CALLS);
        printf("Total concorde calls: %d\n", TOTAL_TSP_CONCORDE_CALLS);
        debug_file.close();

        return path;
    }

}