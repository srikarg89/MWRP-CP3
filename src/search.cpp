#include "search.hpp"
#include "shared.hpp"
#include "utils.hpp"
#include "pathfinding.hpp"
#include "heuristics.hpp"
#include "collisions.hpp"
#include "los.hpp"
#include <queue>
#include <unordered_map>
#include <iostream>
#include <chrono>
#include <boost/functional/hash.hpp> // For boost::hash_value

int NUM_SKIPPED = 0;
int MAX_EXISTING_NODES_SIZE = 0;
double NEIGHBOR_EXPANSION_TIME = 0.0;
double GET_F_VALUE_TIME = 0.0;

using generated_costs_key = std::tuple<std::string, std::string, size_t>;

int get_f_value(HeuristicType heuristic_type, const Map& map, std::vector<AgentState> agent_states, int node_cost, const boost::dynamic_bitset<>& seen, const std::vector<int>& tasks_left, const Lookup& lookup){
    std::vector<int> non_terminated_agent_map_idxs;
    std::vector<int> non_terminated_agent_costs;

    for(const auto& agent : agent_states){
        if(!agent.terminated){
            non_terminated_agent_map_idxs.push_back(map.get_map_idx(agent.pos));
            non_terminated_agent_costs.push_back(agent.cost);
        }
    }

    if(heuristic_type == BFS){
        return node_cost;
    } else if(heuristic_type == SINGLETON) {
        return get_singleton_f_value(non_terminated_agent_map_idxs, non_terminated_agent_costs, node_cost, seen, tasks_left, lookup);
    } else if(heuristic_type == MST || heuristic_type == TSP || heuristic_type == MAX) {
        if(agent_states.size() != 1 && heuristic_type == MST){
            printf("MST heuristic only supports single-agent watchman.\n");
            assert(false);
            exit(1);
        }

        DisjointGraph disjoint_graph = compute_disjoint_graph(lookup, non_terminated_agent_map_idxs, seen, tasks_left);
        // If there's no pivots, just return the singleton heuristic.
        if(disjoint_graph.pivots.size() == 0){
            return get_singleton_f_value(non_terminated_agent_map_idxs, non_terminated_agent_costs, node_cost, seen, tasks_left, lookup);
        }

        prune_graph(disjoint_graph, lookup);

        if(heuristic_type == MST){
            return node_cost + get_mst_heuristic(disjoint_graph);
        } else {
            int tsp_f_value = 0;
            if(agent_states.size() == 1){
                auto [ tsp_heuristic, tsp_path ] = get_tsp_heuristic(disjoint_graph);
                tsp_f_value = node_cost + tsp_heuristic;
            } else {
                int mtsp_f_value = get_multi_tsp_f_value(disjoint_graph, non_terminated_agent_costs);
                // printf("MTSP Heuristic: %d\n", mtsp_f_value);
                tsp_f_value = std::max(node_cost, mtsp_f_value);
            }
            if(heuristic_type == TSP){
                return tsp_f_value;
            } else if(heuristic_type == MAX){
                int singleton_f_value = get_singleton_f_value(non_terminated_agent_map_idxs, non_terminated_agent_costs, node_cost, seen, tasks_left, lookup);
                return std::max(tsp_f_value, singleton_f_value);
            }
        }
    }

    printf("UNKNOWN HEURISTIC TYPE ???\n");
    exit(1);
    return -1;
}

std::vector<std::vector<AgentState>> get_possible_moves(const Map& map, const std::vector<AgentState>& agents, const boost::dynamic_bitset<>& seen, const std::vector<int>& tasks_left, const Lookup& lookup, bool expanding_borders){
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
            std::vector<std::tuple<Position, int>> nbrs_with_added_cost = get_extended_neighbors(map, agent.pos, seen, tasks_left, lookup);
            for(auto [nbr, added_cost] : nbrs_with_added_cost){
                agent_options.push_back(AgentState(nbr, false, agent.cost + added_cost));
            }
        }
        // Generic one-step implementation
        else {
            std::vector<Position> nbrs = map.get_neighbors(agent.pos);
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

std::vector<Node> get_neighbors(Node& node, const Map& map, const Lookup& lookup, SolverConfig solver_config, int last_id_assigned, std::unordered_map<generated_costs_key, int, boost::hash<generated_costs_key>>& generated_costs){
    std::vector<Node> neighbor_nodes;
    auto start = std::chrono::high_resolution_clock::now();
    auto neighbors = get_possible_moves(map, node.agents, node.seen, node.tasks_left, lookup, solver_config.expanding_borders);
    auto end = std::chrono::high_resolution_clock::now();
    std::chrono::duration<double> duration = end - start;
    NEIGHBOR_EXPANSION_TIME += duration.count();
    for(const auto& nbr : neighbors){
        boost::dynamic_bitset<> nbr_seen = node.seen;
        // For each neighbor, loop through path to neighbor.
        int new_squares_seen = 0;
        int nbr_cost = 0;
        std::vector<int> nbr_tasks_left = node.tasks_left;
        for(const AgentState& agent : nbr){
            new_squares_seen += add_los_to_seen(nbr_seen, lookup.los[map.get_map_idx(agent.pos)], map);
            nbr_cost = std::max(nbr_cost, agent.cost);

            // If the agent has reached the task, remove it from the tasks left.
            int agent_map_idx = map.get_map_idx(agent.pos);
            auto it = std::find(nbr_tasks_left.begin(), nbr_tasks_left.end(), agent_map_idx);
            if(it != nbr_tasks_left.end()){
                nbr_tasks_left.erase(it);
            }
        }

        int nbr_num_seen = node.num_seen + new_squares_seen;

        generated_costs_key nbr_key = std::make_tuple(agent_states_to_string(nbr), int_array_to_string(nbr_tasks_left), boost::hash_value(nbr_seen));
        if(generated_costs.find(nbr_key) != generated_costs.end()){
            if(nbr_cost >= generated_costs[nbr_key]){
                // We've already generated a cheaper version of this node.
                NUM_SKIPPED += 1;
                continue;
            }
        }
        generated_costs[nbr_key] = nbr_cost;

        // Calculate heuristic.
        HeuristicType heuristic_type = solver_config.heuristic_type;
        if(heuristic_type == LAZY){
            heuristic_type = SINGLETON;
        }
        auto start = std::chrono::high_resolution_clock::now();
        int nbr_f_value = get_f_value(heuristic_type, map, nbr, nbr_cost, nbr_seen, nbr_tasks_left, lookup);
        if(nbr_f_value < node.f_value){
            printf("ERROR F-VALUE NOT CONSISTENT! F value: %d, %d\n", nbr_f_value, node.f_value);
            printf("Curr node:\n");
            for(const AgentState& a : node.agents){
                printf("\tAgent: %s, terminated: %d, cost: %d\n", a.pos.toString().c_str(), a.terminated, a.cost);
            }
            DisjointGraph disjoint_graph = compute_disjoint_graph(lookup, {map.get_map_idx(node.agents[0].pos), map.get_map_idx(node.agents[1].pos)}, node.seen, {});
            prune_graph(disjoint_graph, lookup);
            for(int i = 0; i < disjoint_graph.pivots.size(); i++){
                int pivot = disjoint_graph.pivots[i];
                if(i < disjoint_graph.num_exploration_pivots){
                    printf("\tExploration pivot: %s\n", map.get_pos_from_map_idx(pivot).toString().c_str());
                } else {
                    printf("\tTask pivot: %s\n", map.get_pos_from_map_idx(pivot).toString().c_str());
                }
            }
            int mtsp_f_value = get_multi_tsp_f_value(disjoint_graph, {node.agents[0].cost, node.agents[1].cost});
            printf("\tMTSP F Value: %d\n", mtsp_f_value);

            printf("Nbr node:\n");
            for(const AgentState& a : nbr){
                printf("\tAgent: %s, terminated: %d, cost: %d\n", a.pos.toString().c_str(), a.terminated, a.cost);
            }
            disjoint_graph = compute_disjoint_graph(lookup, {map.get_map_idx(nbr[0].pos), map.get_map_idx(nbr[1].pos)}, nbr_seen, {});
            prune_graph(disjoint_graph, lookup);
            for(int i = 0; i < disjoint_graph.pivots.size(); i++){
                int pivot = disjoint_graph.pivots[i];
                if(i < disjoint_graph.num_exploration_pivots){
                    printf("\tExploration pivot: %s\n", map.get_pos_from_map_idx(pivot).toString().c_str());
                } else {
                    printf("\tTask pivot: %s\n", map.get_pos_from_map_idx(pivot).toString().c_str());
                }
            }
            mtsp_f_value = get_multi_tsp_f_value(disjoint_graph, {nbr[0].cost, nbr[1].cost});
            printf("\tMTSP F Value: %d\n", mtsp_f_value);

            exit(0);
        }
        auto end = std::chrono::high_resolution_clock::now();
        std::chrono::duration<double> duration = end - start;
        GET_F_VALUE_TIME += duration.count();
        last_id_assigned += 1;
        neighbor_nodes.push_back(Node(last_id_assigned, nbr, nbr_seen, nbr_tasks_left, nbr_cost, nbr_f_value, nbr_num_seen));
    }
    return neighbor_nodes;
}

// Inputs: Agent starting positions, LOS type, map.
// Output: Optimal path.
std::vector<std::vector<Position>> run_search(std::vector<Position> starts, std::vector<Position> incomplete_tasks_pos, boost::dynamic_bitset<> start_seen, const Map& map, const SolverConfig& solver_config, const Lookup& lookup){
    // Create initial seen bitset.
    int num_start_seen = start_seen.count();
    for(int map_idx = 0; map_idx < map.num_squares; map_idx++){
        if(start_seen[map_idx]){
            continue;
        }
        if(lookup.strictly_easier[map_idx] || map.check_obstacle(map.get_pos_from_map_idx(map_idx))){
            start_seen[map_idx] = 1;
            num_start_seen += 1;
        }
    }
    std::vector<AgentState> start_agent_states;
    for(Position start : starts){
        num_start_seen += add_los_to_seen(start_seen, lookup.los[map.get_map_idx(start)], map);
        start_agent_states.push_back(AgentState(start, false, 0));
    }

    // Initialize search data structures.
    std::priority_queue<Node, std::vector<Node>, std::greater<Node>> queue;
    std::unordered_map<int, int> pred_lookup;
    std::unordered_map<int, std::vector<AgentState>> id_lookup;
    std::unordered_set<std::tuple<std::string, size_t>, boost::hash<std::tuple<std::string, size_t>>> visited_nodes; // (map_idx, seen bitset hash)
    std::unordered_map<generated_costs_key, int, boost::hash<generated_costs_key>> generated_costs; // (agent positions, task positions, seen bitset hash)

    std::ofstream debug_file;
    debug_file.open("search_debug.csv");
    debug_file << "Node ID, X, Y, Cost, Heuristic, F Value, Num Seen, Seen Bitset\n"; // Header
    debug_file << map.map_name << "\n";

    std::vector<int> incomplete_tasks;
    for(Position task_pos : incomplete_tasks_pos){
        incomplete_tasks.push_back(map.get_map_idx(task_pos));
    }

    // Setup the starting variable. Mark all obstacles as seen.
    int num_obstacles = 0;
    for(int map_idx = 0; map_idx < map.num_squares; map_idx++){
        if(map.check_obstacle(map.get_pos_from_map_idx(map_idx))){
            num_obstacles += 1;
        }
    }
    int num_free = map.num_squares - num_obstacles;

    HeuristicType start_heuristic_type = solver_config.heuristic_type;
    if(start_heuristic_type == LAZY){
        start_heuristic_type = SINGLETON;
    }
    printf("Incomplete tasks (%ld): ", incomplete_tasks.size());
    for(int task : incomplete_tasks){
        printf("%s ", map.get_pos_from_map_idx(task).toString().c_str());
    }
    printf("\n");
    int start_f_value = get_f_value(start_heuristic_type, map, start_agent_states, 0, start_seen, incomplete_tasks, lookup);
    printf("Start f value: %d\n", start_f_value);
    queue.push(Node(/* id = */ 0, start_agent_states, start_seen, incomplete_tasks, /* cost = */ 0, start_f_value, num_start_seen));

    std::vector<Node> expanded_nodes;

    int num_expanded = 0;
    int num_generated = 0;
    int last_id_assigned = 0;
    int max_new_squares_seen = 0;
    int num_skipped = 0;
    int num_skipped_dom = 0;
    int num_fully_expanded = 0;

    auto start_time = std::chrono::high_resolution_clock::now();

    std::vector<std::vector<Position>> paths(starts.size(), std::vector<Position>());

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

        num_expanded += 1;

        if(solver_config.heuristic_type == LAZY && curr.is_lazy){
            // Recompute f value.
            int new_f_value = get_f_value(HeuristicType::TSP, map, curr.agents, curr.cost, curr.seen, curr.tasks_left, lookup);
            new_f_value = std::max(new_f_value, curr.f_value); // Ensure f value never decreases.
            curr.update_f_value(new_f_value);
            queue.push(curr);
            continue;
        }

        num_fully_expanded += 1;

        expanded_nodes.push_back(curr);

        visited_nodes.insert(visited_key);
        id_lookup[curr.node_id] = curr.agents;

        write_node_to_file(debug_file, curr, lookup, map, pred_lookup[curr.node_id], solver_config.heuristic_type);

        // debug_file.close(); exit(0);

        max_new_squares_seen = std::max(max_new_squares_seen, curr.num_seen - num_obstacles);
        if(num_fully_expanded % 100 == 0){
            printf("Expanded %d nodes. Fully expanded %d nodes. Num generated %d. Loc: %s, cost: %d, heuristic: %d, num free seen: %d / %d, max free squares seen: %d\n", num_expanded, num_fully_expanded, num_generated, agent_states_to_print_string(curr.agents).c_str(), curr.cost, curr.heuristic, (curr.num_seen - num_obstacles), num_free, max_new_squares_seen);
            printf("\tF value: %d. Cost: %d. Heuristic: %d\n", curr.f_value, curr.cost, curr.heuristic);
            // printf("\tQueue size: %ld. Visited size: %ld. Generated costs size: %ld. Max existing nodes size: %d. Num skipped: %d\n", queue.size(), visited_nodes.size(), generated_costs.size(), MAX_EXISTING_NODES_SIZE, num_skipped);
        }

        // printf("Expanding node %d. Node ID: %d, Loc: %s, cost: %d, heuristic: %d, num seen: %d\n", num_expanded, curr.node_id, agent_states_to_print_string(curr.agents).c_str(), curr.cost, curr.heuristic, curr.num_seen);

        // Goal condition.
        if(curr.num_seen == map.num_squares && curr.tasks_left.size() == 0){
            printf("Goal condition met!\n");
            printf("Num seen: %d / %d\n", curr.num_seen, map.num_squares);
            for(int i = 0; i < curr.seen.size(); i++){
                if(!curr.seen[i]){
                    printf("UNSEEN: %s\n", map.get_pos_from_map_idx(i).toString().c_str());
                }
            }
            auto end_time = std::chrono::high_resolution_clock::now();
            auto seconds_taken = std::chrono::duration<double>(end_time - start_time).count();
            printf("Search time taken: %.3f seconds\n", seconds_taken);
            printf("Solution cost: %d\n", curr.cost);

            int curr_id = curr.node_id;
            std::vector<Position> curr_positions = agent_states_to_positions(id_lookup[curr_id]);
            while(curr_id != 0){
                curr_id = pred_lookup[curr_id];
                std::vector<Position> prev_positions = agent_states_to_positions(id_lookup[curr_id]);
                for(int i = 0; i < paths.size(); i++){
                    int idx = map.get_map_idx(curr_positions[i]);
                    int from_idx = map.get_map_idx(prev_positions[i]);
                    while(idx != from_idx){
                        paths[i].push_back(map.get_pos_from_map_idx(idx));
                        idx = lookup.apsp_paths[from_idx][idx];
                    }
                }
                curr_positions = prev_positions;
            }

            for(int i = 0; i < paths.size(); i++){
                paths[i].push_back(starts[i]);
            }

            for(int i = 0; i < paths.size(); i++){
                std::reverse(paths[i].begin(), paths[i].end());
                printf("Path for agent (length %ld) %d: ", paths[i].size(), i);
                for(Position pos : paths[i]){
                    printf("%s ", pos.toString().c_str());
                }
                printf("\n");
            }
            break;
        }

        std::vector<Node> neighbors = get_neighbors(curr, map, lookup, solver_config, last_id_assigned, generated_costs);
        num_generated += neighbors.size();
        if(neighbors.size() > 0){
            last_id_assigned = neighbors.back().node_id;
        }
        for(Node& nbr : neighbors){
            // printf("\tGenerated neighbor. Node ID: %d, Loc: %s, cost: %d, heuristic: %d, num seen: %d\n", nbr.node_id, nbr.pos.toString().c_str(), nbr.cost, nbr.heuristic, nbr.num_seen);
            pred_lookup[nbr.node_id] = curr.node_id;
            if(nbr.f_value < curr.f_value){
                printf("ERROR?? F value decreased from %d to %d\n", curr.f_value, nbr.f_value);
                exit(0);
            }
            queue.push(nbr);
        }
    }

    if(solver_config.collision_resolution == POSTPROCESS){
        paths = postprocess_collisions(paths);
    }
    add_waits_to_end(paths);

    printf("Total nodes expanded: %d\n", num_expanded);
    printf("Total nodes fully expanded: %d\n", num_fully_expanded);
    printf("Total expansions skipped: %d\n", num_skipped);
    // printf("Total expansions skipped by domination check: %d\n", num_skipped_dom);
    printf("Total generations skipped: %d\n", NUM_SKIPPED);
    printf("Total nodes generated: %d\n", num_generated);
    if(solver_config.heuristic_type == TSP || solver_config.heuristic_type == MAX || solver_config.heuristic_type == LAZY){
        if(start_agent_states.size() == 1){
            printf("Total heuristic time: %.3f seconds\n", TOTAL_HEURISTIC_TIME);
        } else{
            printf("MTSP Setup time: %.3f seconds\n", TOTAL_RUNTIME);
            printf("MTSP Solver time: %.3f seconds\n", SOLVER_RUNTIME);
            printf("Total MTSP calls: %d\n", TOTAL_CALLS);
        }
    }
    printf("Total neighbor expansion time: %.3f seconds\n", NEIGHBOR_EXPANSION_TIME);
    printf("Total get_f_value time: %.3f seconds\n", GET_F_VALUE_TIME);
    for(int i = 0; i < paths.size(); i++){
        printf("Path %d length: %ld\n", i, paths[i].size());
    }

    debug_file.close();

    std::ofstream solution_file;
    solution_file.open("search_solution.csv");
    solution_file << "Timestep, Num Agents, Num seen, Agent positions, Seen Bitset\n"; // Header
    solution_file << map.map_name << "\n";
    write_solution_to_file(solution_file, paths, start_seen, map, lookup);
    solution_file.close();

    return paths;
}


/*
EB

Start f value: 22
Goal condition met!
Num seen: 121 / 121
Search time taken: 0.968 seconds
Solution cost: 25
Path for agent (length 24) 0: (0,0) (0,1) (1,1) (2,1) (2,0) (3,0) (4,0) (4,1) (4,2) (4,3) (5,3) (6,3) (7,3) (8,3) (9,3) (10,3) (9,3) (8,3) (7,3) (6,3) (6,2) (6,1) (7,1) (7,0) 
Path for agent (length 26) 1: (10,10) (9,10) (8,10) (8,9) (7,9) (7,8) (7,7) (7,6) (7,5) (7,6) (7,7) (7,8) (6,8) (5,8) (5,9) (5,10) (5,9) (5,8) (4,8) (3,8) (3,7) (2,7) (1,7) (1,6) (1,5) (0,5) 
Total nodes expanded: 36
Total nodes fully expanded: 36
Total expansions skipped: 0
Total generations skipped: 32
Total nodes generated: 359
Total heuristic time: 0.000 seconds
Total MTSP time: 0.057 seconds
MTSP Solver time: 0.918 seconds
Total MTSP calls: 359
Solution size: 2


Search time taken: 1.021 seconds
Solution cost: 25
Path for agent (length 24) 0: (0,0) (0,1) (1,1) (2,1) (2,0) (3,0) (4,0) (4,1) (4,2) (4,3) (5,3) (6,3) (7,3) (8,3) (9,3) (10,3) (9,3) (8,3) (7,3) (6,3) (6,2) (6,1) (7,1) (7,0) 
Path for agent (length 26) 1: (10,10) (9,10) (8,10) (8,9) (7,9) (7,8) (7,7) (7,6) (7,5) (7,6) (7,7) (7,8) (6,8) (5,8) (5,9) (5,10) (5,9) (5,8) (4,8) (3,8) (3,7) (2,7) (1,7) (1,6) (1,5) (0,5) 
Total nodes expanded: 36
Total nodes fully expanded: 36
Total expansions skipped: 0
Total generations skipped: 32
Total nodes generated: 359
MTSP Setup time: 0.063 seconds
MTSP Solver time: 0.960 seconds
Total MTSP calls: 359

*/