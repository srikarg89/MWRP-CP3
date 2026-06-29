#define BS_THREAD_POOL_NATIVE_EXTENSIONS
#include "search.hpp"
#include "shared.hpp"
#include "utils.hpp"
#include "heuristics.hpp"
#include "los.hpp"
#include <unordered_map>
#include <iostream>
#include <chrono>
#include <boost/functional/hash.hpp> // For boost::hash_value
#include <boost/heap/fibonacci_heap.hpp>
#include "BS_thread_pool.hpp"

std::vector<std::pair<int, int>> get_f_and_focal_values(HeuristicType heuristic_type, FocalMethod focal_method, Optimizations optimizations, const Map& map, const std::vector<HeuristicInput>& neighbor_heuristic_inputs, double focal_heuristic_weight, const Lookup& lookup) {
    std::vector<std::pair<int, int>> f_and_focal_values; // Minimum f value is the node cost (no such thing as a negative heuristic).
    for (const auto& input : neighbor_heuristic_inputs) {
        f_and_focal_values.push_back(std::make_pair(input.cost, 0));
    }

    if(heuristic_type == BFS){
        return f_and_focal_values;
    }

    // Used for parallel TSP heuristic computation.
    BS::thread_pool pool;
    std::vector<std::future<std::pair<int, int>>> futures;
    std::vector<int> tsp_idxs;
    std::vector<std::pair<int, int>> non_parallel_results;

    for(int i = 0; i < neighbor_heuristic_inputs.size(); i++) {
        const auto& input = neighbor_heuristic_inputs[i];
        std::vector<AgentState> non_terminated_agents;

        for(const auto& agent : input.agents){
            if(!agent.terminated){
                non_terminated_agents.push_back(agent);
            }
        }

        bool use_singleton = (heuristic_type == SINGLETON || heuristic_type == LAZY || heuristic_type == MAX);

        if(heuristic_type == TSP || heuristic_type == MAX || heuristic_type == LAZY) {
            DisjointGraph disjoint_graph = compute_disjoint_graph(map, non_terminated_agents, input.seen, lookup, optimizations.max_pivots_generated);
            if(optimizations.prune_pivots){
                prune_graph(disjoint_graph, lookup);
            }
            // printf("Num pivots: %d\n", (int)disjoint_graph.pivots.size());

            // If there's no pivots, just return the singleton heuristic.
            if(disjoint_graph.pivots.size() == 0){
                use_singleton = true;
            } else {
                std::vector<int> non_terminated_agent_costs;
                for(const auto& agent : non_terminated_agents){
                    non_terminated_agent_costs.push_back(agent.cost);
                }
                tsp_idxs.push_back(i);
                if(optimizations.run_parallel){
                    futures.push_back(pool.submit_task([disjoint_graph, non_terminated_agent_costs, focal_method, focal_heuristic_weight]() {
                        // if(focal_heuristic_weight == 1.0) {
                        //     return get_multi_tsp_f_and_focal_value(disjoint_graph, non_terminated_agent_costs, focal_method, false);
                        // }
                        // auto a = get_multi_tsp_f_and_focal_value(disjoint_graph, non_terminated_agent_costs, focal_method, false);
                        // auto b = get_multi_tsp_f_and_focal_value(disjoint_graph, non_terminated_agent_costs, focal_method, true);
                        // return std::make_pair(a.first, b.second);
                        return get_multi_tsp_f_and_focal_value(disjoint_graph, non_terminated_agent_costs, focal_method, false);
                    }));
                } else {
                    // if(focal_heuristic_weight == 1.0) {
                    //     non_parallel_results.push_back(get_multi_tsp_f_and_focal_value(disjoint_graph, non_terminated_agent_costs, focal_method, false));
                    // }
                    // auto a = get_multi_tsp_f_and_focal_value(disjoint_graph, non_terminated_agent_costs, focal_method, false);
                    // auto b = get_multi_tsp_f_and_focal_value(disjoint_graph, non_terminated_agent_costs, focal_method, true);
                    // non_parallel_results.push_back(std::make_pair(a.first, b.second));
                    non_parallel_results.push_back(get_multi_tsp_f_and_focal_value(disjoint_graph, non_terminated_agent_costs, focal_method, false));
                }
            }
        }

        // Singleton heuristic.
        if(use_singleton) {
            int singleton_val = get_singleton_f_value(input.agents, map, input.cost, input.seen, lookup);
            f_and_focal_values[i].first = std::max(f_and_focal_values[i].first, singleton_val);
            std::vector<AgentState> agents_for_focal = input.agents;
            for(auto& agent : agents_for_focal){
                agent.cost = 0;
            }
            f_and_focal_values[i].second = get_singleton_f_value(agents_for_focal, map, 0, input.seen, lookup);
        }
    }

    // Collect TSP/MAX heuristic results.
    for(int i = 0; i < tsp_idxs.size(); i++) {
        auto result = optimizations.run_parallel ? futures[i].get() : non_parallel_results[i];
        f_and_focal_values[tsp_idxs[i]].first = std::max(f_and_focal_values[tsp_idxs[i]].first, result.first); // Ensure that f value is max of singleton and TSP.
        f_and_focal_values[tsp_idxs[i]].second = std::max(f_and_focal_values[tsp_idxs[i]].second, result.second); // Ensure that focal value is max of singleton and TSP.
    }

    // Weighted A* for focal heuristic, but with sum of costs instead of makespan.
    for(int i = 0; i < f_and_focal_values.size(); i++) {
        double current_sum = 0.0;
        if(focal_method == FocalMethod::SOC) {
            for(const auto& agent : neighbor_heuristic_inputs[i].agents){
                current_sum += agent.cost;
            }
        } else {
            for(const auto& agent : neighbor_heuristic_inputs[i].agents){
                current_sum = std::max(current_sum, (double)agent.cost);
            }
        }
        f_and_focal_values[i].second = current_sum + focal_heuristic_weight * f_and_focal_values[i].second;
    }

    return f_and_focal_values;
}

std::vector<AgentState> get_agent_options(const AgentState& agent, const Map& map, const boost::dynamic_bitset<>& seen, const Lookup& lookup, bool allow_terminate) {
    if(agent.terminated){
        // If any agent is terminated, we don't generate any neighbors.
        return {agent};
    }
    std::vector<AgentState> agent_options;

    // Expanding borders implementation.
    std::vector<std::tuple<Position, int>> nbrs_with_added_cost = get_extended_neighbors(map, agent.pos, seen, lookup);
    for(auto [nbr, added_cost] : nbrs_with_added_cost){
        agent_options.push_back(AgentState(nbr, false, agent.cost + added_cost));
    }

    if(allow_terminate){
        agent_options.push_back(AgentState(agent.pos, true, agent.cost)); // Option to terminate.
    }
    return agent_options;
}


// Expanding Borders Expansion
std::vector<std::vector<AgentState>> get_possible_moves(const Map& map, const std::vector<AgentState>& agents, const boost::dynamic_bitset<>& seen, const Lookup& lookup){
    std::vector<std::vector<AgentState>> options;
    for(AgentState agent : agents){
        options.push_back(get_agent_options(agent, map, seen, lookup, true));
    }

    // Now, take the cartesian product of the options. Don't include states in which all of the agents terminate.
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

std::vector<Node> get_neighbors(Node& node, const Map& map, const Lookup& lookup, SolverConfig solver_config, int best_solution_cost, int last_id_assigned, std::unordered_map<std::string, std::vector<VisitedNodeInfo>>& generated_costs, std::unordered_set<int>& avoid_expansion_list){
    std::vector<Node> neighbor_nodes;
    auto start = std::chrono::high_resolution_clock::now();
    auto neighbors = get_possible_moves(map, node.agents, node.seen, lookup);
    auto end = std::chrono::high_resolution_clock::now();
    std::chrono::duration<double> duration = end - start;
    METRICS.neighbor_expansion_time += duration.count();

    std::vector<HeuristicInput> neighbor_heuristic_inputs;

    for(auto& nbr : neighbors){
        boost::dynamic_bitset<> nbr_seen = node.seen;
        // For each neighbor, loop through path to neighbor.
        int nbr_num_seen = node.num_seen;
        int nbr_cost = 0;
        start = std::chrono::high_resolution_clock::now();
        for(const AgentState& agent : nbr){
            nbr_num_seen += add_los_to_seen(nbr_seen, lookup.los[map.get_map_idx(agent.pos)], map);
            nbr_cost = std::max(nbr_cost, agent.cost);
        }
        end = std::chrono::high_resolution_clock::now();
        duration = end - start;
        METRICS.adding_los_time += duration.count();

        std::string nbr_key = agent_states_to_string(nbr);
        std::vector<AgentState> nbr_sorted = get_sorted_agents_by_position(nbr);

        // Check for dominance in generated costs.
        start = std::chrono::high_resolution_clock::now();
        bool dominated = false;
        int dominated_node_id = -1;
        // Current node is dominated by existing node, skip.
        if(generated_costs.find(nbr_key) != generated_costs.end()){
            auto& visited_node_list = generated_costs[nbr_key];
            auto it = visited_node_list.begin();
            while (it != visited_node_list.end()) {
                bool nbr_cost_better = true;
                bool visited_cost_better = true;
                for(int i = 0; i < nbr_sorted.size(); i++){
                    if((it->agents[i].cost < nbr_sorted[i].cost)){
                        nbr_cost_better = false;
                    } else if((nbr_sorted[i].cost < it->agents[i].cost)){
                        visited_cost_better = false;
                    }
                }

                if(visited_cost_better && nbr_seen.is_subset_of(it->seen)){
                    // Existing node dominates current node, skip current node.
                    METRICS.num_skipped_duplicate_node += 1;
                    dominated = true;
                    dominated_node_id = it->id;
                    break;
                } else if(nbr_cost_better && it->seen.is_subset_of(nbr_seen)){
                    // Current node dominates existing node, remove existing node.
                    avoid_expansion_list.insert(it->id);
                    it = visited_node_list.erase(it);
                } else {
                    ++it;
                }
            }
        }

        end = std::chrono::high_resolution_clock::now();
        duration = end - start;
        METRICS.domination_check_time += duration.count();

        if(dominated){
            continue;
        }

        if(solver_config.heuristic_type == TSP){
            double min_nbr_f_value = get_singleton_f_value(nbr, map, nbr_cost, nbr_seen, lookup);
            if(min_nbr_f_value >= best_solution_cost){
                // Don't generate neighbors that are already worse than the best solution found so far.
                METRICS.num_skipped_high_lazy_f_value += 1;
                continue;
            }
        }

        neighbor_heuristic_inputs.push_back(HeuristicInput{nbr, nbr_cost, nbr_seen, nbr_num_seen});
    }

    HeuristicType heuristic_type = solver_config.heuristic_type;
    if(heuristic_type == LAZY){
        heuristic_type = SINGLETON;
    }

    start = std::chrono::high_resolution_clock::now();
    std::vector<std::pair<int, int>> f_and_focal_values = get_f_and_focal_values(heuristic_type, solver_config.focal_method, solver_config.optimizations, map, neighbor_heuristic_inputs, solver_config.focal_heuristic_weight, lookup);
    end = std::chrono::high_resolution_clock::now();
    duration = end - start;
    METRICS.lazy_f_value_calculation_time += duration.count();

    for(int i = 0; i < neighbor_heuristic_inputs.size(); i++){
        const HeuristicInput& input = neighbor_heuristic_inputs[i];

        // Weighted A* for 
        int nbr_f_value = f_and_focal_values[i].first;

        // Apply pathmax to ensure consistency.
        int node_admissible_f_value = std::max((int)((double)node.f_value / A_STAR_WEIGHT), node.cost);
        // int node_admissible_f_value = node.f_value;
        nbr_f_value = std::max(node_admissible_f_value, nbr_f_value);
        int nbr_focal_value = f_and_focal_values[i].second;
        last_id_assigned += 1;

        std::string nbr_key = agent_states_to_string(input.agents);
        std::vector<AgentState> nbr_sorted = get_sorted_agents_by_position(input.agents);
        if(generated_costs.find(nbr_key) == generated_costs.end()){
            generated_costs[nbr_key] = std::vector<VisitedNodeInfo>{};
        }
        generated_costs[nbr_key].push_back(VisitedNodeInfo{last_id_assigned, nbr_sorted, input.seen});

        neighbor_nodes.push_back(Node(last_id_assigned, input.agents, input.seen, input.cost, nbr_f_value, nbr_focal_value, input.num_seen, node.depth + 1));
    }

    return neighbor_nodes;
}

std::vector<std::vector<Position>> reconstruct_path(int goal_node_id, const std::unordered_map<int, int>& pred_lookup, const std::unordered_map<int, std::vector<AgentState>>& id_lookup, const std::vector<Position>& starts, const Map& map, const Lookup& lookup){
    int curr_id = goal_node_id;
    std::vector<AgentState> curr_agent_states = id_lookup.at(curr_id);
    std::vector<Position> curr_positions = agent_states_to_positions(curr_agent_states);
    std::vector<std::vector<Position>> paths(starts.size(), std::vector<Position>());
    while(curr_id != 0){
        curr_id = pred_lookup.at(curr_id);
        std::vector<AgentState> prev_agent_states = id_lookup.at(curr_id);
        std::vector<Position> prev_positions = agent_states_to_positions(prev_agent_states);
        for(int i = 0; i < paths.size(); i++){
            int idx = map.get_map_idx(curr_positions[i]);
            int from_idx = map.get_map_idx(prev_positions[i]);
            // Add waits into path if it was a wait action.
            if(idx == from_idx) {
                for(int t = prev_agent_states[i].cost; t < curr_agent_states[i].cost; t++){
                    paths[i].push_back(prev_positions[i]);
                }
                continue;
            }
            while(idx != from_idx){
                paths[i].push_back(map.get_pos_from_map_idx(idx));
                idx = lookup.apsp_paths[from_idx][idx];
            }
        }
        curr_positions = prev_positions;
        curr_agent_states = prev_agent_states;
    }

    for(int i = 0; i < paths.size(); i++){
        paths[i].push_back(starts[i]);
        std::reverse(paths[i].begin(), paths[i].end());
    }
    return paths;
}

std::vector<HeuristicInput> get_heuristic_inputs_from_nodes(const std::vector<Node>& nodes){
    std::vector<HeuristicInput> inputs;
    for(const Node& node : nodes){
        inputs.push_back(HeuristicInput{node.agents, node.cost, node.seen, node.num_seen});
    }
    return inputs;
}


// Output: Optimal makespan path.
std::vector<std::vector<Position>> run_search(int start_timestep, std::vector<Position> starts, boost::dynamic_bitset<> start_seen, const Map& map, const SolverConfig& solver_config, const Lookup& lookup) {
    // Reset metrics for every search run.
    METRICS.reset();

    // Create initial seen bitset.
    add_dominance_to_seen(start_seen, map, lookup);
    int num_start_seen = start_seen.count();
    
    std::vector<AgentState> start_agent_states;
    for(Position start : starts){
        num_start_seen += add_los_to_seen(start_seen, lookup.los[map.get_map_idx(start)], map);
        start_agent_states.push_back(AgentState(start, false, start_timestep));
    }

    // Initialize search data structures.
    using Heap = boost::heap::fibonacci_heap<Node, boost::heap::compare<std::greater<Node>>>;
    Heap open_set;
    std::unordered_map<int, Heap::handle_type> handle_lookup;

    boost::heap::fibonacci_heap<Node, boost::heap::compare<CompareNodeFocal>> focal_list;
    int prev_min_f = -1;
    std::unordered_set<int> added_to_focal_list;

    std::unordered_map<int, int> pred_lookup;
    std::unordered_map<int, std::vector<AgentState>> id_lookup;
    std::unordered_map<std::string, std::vector<VisitedNodeInfo>> generated_costs;
    std::unordered_set<int> avoid_expansion_list;

    std::ofstream debug_file;
    debug_file.open("search_debug.csv");
    debug_file << "Node ID, X, Y, Cost, Heuristic, F Value, Num Seen, Seen Bitset\n"; // Header
    debug_file << map.map_name << "\n";

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

    auto [start_f_value, start_focal_value] = get_f_and_focal_values(start_heuristic_type, solver_config.focal_method, solver_config.optimizations, map, {HeuristicInput{start_agent_states, start_timestep, start_seen, num_start_seen}}, solver_config.focal_heuristic_weight, lookup)[0];
    printf("Start f value: %d, Start focal value: %d, Num start seen: %d / %d\n", start_f_value, start_focal_value, num_start_seen, map.num_squares);

    handle_lookup[0] = open_set.push(Node(/* id = */ 0, start_agent_states, start_seen, /* cost = */ start_timestep, start_f_value, start_focal_value, num_start_seen, /*depth = */ 0));

    std::vector<Node> expanded_nodes;

    int num_generated = 0;
    int last_id_assigned = 0;
    int max_new_squares_seen = 0;
    int num_skipped = 0;
    int num_fully_expanded = 0;
    int num_discarded_high_f = 0;

    auto start_time = std::chrono::high_resolution_clock::now();

    std::vector<std::vector<Position>> solution_paths(starts.size(), std::vector<Position>());
    bool solution_found = false;
    int max_node_depth_expanded = 0;
    int best_solution_cost = INT_MAX;
    int num_lazy_batches_run = 0;

    while(!(open_set.empty())){
        double time_elapsed = std::chrono::duration<double>(std::chrono::high_resolution_clock::now() - start_time).count();

        if(time_elapsed > solver_config.hard_search_time_limit){
            printf("Hard time limit reached, exiting!.\n");
            exit(0);
        }

        if(solution_found && (time_elapsed > solver_config.search_time_limit)){
            printf("Soft time limit reached, ending search.\n");
            break;
        }

        // Regular A* vs. Focal A*
        if(solver_config.focal_epsilon == 1.0) {
            // Regular A*
            while(solver_config.heuristic_type == LAZY && open_set.top().is_lazy) {
                int count = 0;
                std::vector<Node> batch_nodes;
                auto it = open_set.ordered_begin();
                int parallel_batch_size = solver_config.optimizations.run_parallel ? solver_config.optimizations.parallel_batch_size : 1;
                if(parallel_batch_size > 0){
                    while(it != open_set.ordered_end() && count < parallel_batch_size){
                        if(it->is_lazy){
                            batch_nodes.push_back(*it);
                        }
                        count++;
                        ++it;
                    }
                } else {
                    double top_f_value = open_set.top().f_value;
                    while(it != open_set.ordered_end() && it->f_value <= top_f_value){
                        if(it->is_lazy){
                            batch_nodes.push_back(*it);
                        }
                        count++;
                        ++it;
                    }
                }

                // printf("Running MxWA* batch heuristic on %d nodes!\n", (int)batch_nodes.size());
                num_lazy_batches_run += 1;
                auto batch_inputs = get_heuristic_inputs_from_nodes(batch_nodes);
                auto f_start = std::chrono::high_resolution_clock::now();
                std::vector<std::pair<int, int>> f_and_focal_values = get_f_and_focal_values(HeuristicType::TSP, solver_config.focal_method, solver_config.optimizations, map, batch_inputs, solver_config.focal_heuristic_weight, lookup);
                auto f_end = std::chrono::high_resolution_clock::now();
                std::chrono::duration<double> f_duration = f_end - f_start;
                METRICS.tsp_f_value_calculation_time += f_duration.count();

                for(int i = 0; i < batch_nodes.size(); i++){
                    Node node = batch_nodes[i];
                    int new_f_value = std::max(f_and_focal_values[i].first, node.f_value); // Ensure f value never decreases.
                    node.update_f_value(new_f_value);
                    node.update_focal_heuristic(f_and_focal_values[i].second);
                    open_set.update(handle_lookup[node.node_id], node);
                }
            }
        } else {
            // Focal A*
            int min_f_value = open_set.top().f_value;
            while(min_f_value > prev_min_f){
                auto it = open_set.ordered_begin();
                int max_f_value = (int)std::ceil(solver_config.focal_epsilon * (double)min_f_value);
                std::vector<Node> nodes_to_recompute;
                while(it != open_set.ordered_end() && it->f_value <= max_f_value){
                    if(it->is_lazy) {
                        nodes_to_recompute.push_back(*it);
                    } else if(added_to_focal_list.find(it->node_id) == added_to_focal_list.end()){
                        focal_list.push(*it);
                        added_to_focal_list.insert(it->node_id);
                    }

                    ++it;
                }

                if(nodes_to_recompute.size() > 0){
                    num_lazy_batches_run += 1;
                    // printf("Running focal batch heuristic on %d nodes!\n", (int)nodes_to_recompute.size());
                    auto batch_inputs = get_heuristic_inputs_from_nodes(nodes_to_recompute);
                    auto f_start = std::chrono::high_resolution_clock::now();
                    std::vector<std::pair<int, int>> f_and_focal_values = get_f_and_focal_values(HeuristicType::TSP, solver_config.focal_method, solver_config.optimizations, map, batch_inputs, solver_config.focal_heuristic_weight, lookup);
                    auto f_end = std::chrono::high_resolution_clock::now();
                    std::chrono::duration<double> f_duration = f_end - f_start;
                    METRICS.tsp_f_value_calculation_time += f_duration.count();
                    for(int i = 0; i < nodes_to_recompute.size(); i++){
                        // Update the node's attributes.
                        Node node = nodes_to_recompute[i];
                        int new_f_value = std::max(f_and_focal_values[i].first, node.f_value); // Ensure f value never decreases.
                        int new_focal_value = f_and_focal_values[i].second;
                        node.update_f_value(new_f_value);
                        node.update_focal_heuristic(new_focal_value);

                        // Add node back into open set and focal list if needed.
                        open_set.update(handle_lookup[node.node_id], node);
                        if(node.f_value <= max_f_value && added_to_focal_list.find(node.node_id) == added_to_focal_list.end()){
                            focal_list.push(node);
                            added_to_focal_list.insert(node.node_id);
                        }
                    }
                }
                    
                prev_min_f = min_f_value;
                min_f_value = open_set.top().f_value;
            }
        }

        // Regular A* vs. Focal A*
        Node curr = (solver_config.focal_epsilon == 1.0) ? open_set.top() : focal_list.top();
        if(solver_config.focal_epsilon == 1.0) {
            // Regular A* - Get lowest cost node in open set.
            open_set.pop();
        } else {
            // Focal A* - Get lowest cost node by focal heuristic and remove from open set.
            focal_list.pop();
            open_set.erase(handle_lookup[curr.node_id]);
        }

        // Skip expansion if we've found a better solution.
        int admissible_f_value = std::max((int)((double)curr.f_value / A_STAR_WEIGHT), curr.cost);
        if(admissible_f_value >= best_solution_cost){
            continue;
        }

        if(avoid_expansion_list.find(curr.node_id) != avoid_expansion_list.end()){
            // This node has been dominated by a node generated after it, so skip its expansion.
            num_skipped += 1;
            continue;
        }

        // printf("Fully expanding node: ID: %d, Loc: %s, Cost: %d, F value: %d, Focal heuristic: %d, Num seen: %d / %d\n", curr.node_id, agent_states_to_print_string(curr.agents).c_str(), curr.cost, curr.f_value, curr.focal_heuristic, curr.num_seen, map.num_squares);

        num_fully_expanded += 1;

        expanded_nodes.push_back(curr);
        max_node_depth_expanded = std::max(max_node_depth_expanded, curr.depth);

        id_lookup[curr.node_id] = curr.agents;

        write_node_to_file(debug_file, curr, lookup, map, pred_lookup[curr.node_id], solver_config.heuristic_type);

        max_new_squares_seen = std::max(max_new_squares_seen, curr.num_seen - num_obstacles);
        if(num_fully_expanded % 100 == 0){
            printf("Fully expanded %d nodes. Num generated %d. Loc: %s, cost: %d, heuristic: %d, num free seen: %d / %d, max free squares seen: %d. Time elapsed: %f\n", num_fully_expanded, num_generated, agent_states_to_print_string(curr.agents).c_str(), curr.cost, curr.heuristic, (curr.num_seen - num_obstacles), num_free, max_new_squares_seen, std::chrono::duration<double>(std::chrono::high_resolution_clock::now() - start_time).count());
            printf("\tF value: %d. Cost: %d. Heuristic: %d. Focal: %d\n", curr.f_value, curr.cost, curr.heuristic, curr.focal_heuristic);
            printf("\tNode depth: %d, Max node depth expanded: %d. Min f value: %d, Max f value searching: %d\n", curr.depth, max_node_depth_expanded, prev_min_f, (int)(solver_config.focal_epsilon * prev_min_f));
        }

        // Goal condition.
        if(curr.num_seen == map.num_squares){
            printf("Goal condition met!\n");
            printf("\tNum seen: %d / %d\n", curr.num_seen, map.num_squares);
            printf("\tSolution node depth: %d\n", curr.depth);
            printf("\tSolution cost: %d. Solution cost minus start: %d\n", curr.cost, curr.cost - start_timestep);
            for(AgentState agent : curr.agents){
                printf("\t\tAgent final time: %d\n", agent.cost);
            }

            solution_found = true;

            if(curr.cost < best_solution_cost) {
                printf("Writing solution to file. Previous best cost: %d, New best cost: %d\n", best_solution_cost, curr.cost);
                double time = std::chrono::duration<double>(std::chrono::high_resolution_clock::now() - start_time).count();
                printf("Anytime solution found at time %f with cost %d\n", time, curr.cost);
                best_solution_cost = curr.cost;
                solution_paths = reconstruct_path(curr.node_id, pred_lookup, id_lookup, starts, map, lookup);
            }

            continue;
            // break;
        }

        std::vector<Node> neighbors = get_neighbors(curr, map, lookup, solver_config, best_solution_cost, last_id_assigned, generated_costs, avoid_expansion_list);

        num_generated += neighbors.size();
        if(neighbors.size() > 0){
            last_id_assigned = neighbors.back().node_id;
        }
        std::vector<Node> batch_nodes;
        for(Node& nbr : neighbors){
            int nbr_admissible_f_value = std::max((int)((double)nbr.f_value / A_STAR_WEIGHT), nbr.cost);
            if(nbr_admissible_f_value >= best_solution_cost){
                num_discarded_high_f += 1;
                continue;
            }

            pred_lookup[nbr.node_id] = curr.node_id;
            // Regular A* vs. Focal A*
            if(solver_config.focal_epsilon == 1.0){
                // Regular A*
                handle_lookup[nbr.node_id] = open_set.push(nbr);
            } else {
                // Focal A*
                int max_f_value = (int)std::ceil(solver_config.focal_epsilon * (double)prev_min_f);
                if(nbr.f_value <= max_f_value && nbr.is_lazy){
                    batch_nodes.push_back(nbr);
                } else {
                    // If not within focal threshold or not running Lazy A* add the node to the open set.
                    handle_lookup[nbr.node_id] = open_set.push(nbr);
                }
            }
        }

        if(batch_nodes.size() > 0){
            num_lazy_batches_run += 1;
            // printf("Running expansion batch heuristic on %d nodes!\n", (int)batch_nodes.size());
            auto batch_inputs = get_heuristic_inputs_from_nodes(batch_nodes);
            int max_f_value = (int)std::ceil(solver_config.focal_epsilon * (double)prev_min_f);
            auto f_start = std::chrono::high_resolution_clock::now();
            std::vector<std::pair<int, int>> f_and_focal_values = get_f_and_focal_values(HeuristicType::TSP, solver_config.focal_method, solver_config.optimizations, map, batch_inputs, solver_config.focal_heuristic_weight, lookup);
            auto f_end = std::chrono::high_resolution_clock::now();
            std::chrono::duration<double> f_duration = f_end - f_start;
            METRICS.tsp_f_value_calculation_time += f_duration.count();
            for(int i = 0; i < batch_nodes.size(); i++){
                // Update the node's attributes.
                Node nbr = batch_nodes[i];
                nbr.update_f_value(std::max(f_and_focal_values[i].first, nbr.f_value)); // Ensure f value never decreases.
                nbr.update_focal_heuristic(std::max(f_and_focal_values[i].second, nbr.focal_heuristic)); // Ensure focal value never decreases.

                // Add node into open set and focal list if needed.
                handle_lookup[nbr.node_id] = open_set.push(nbr);
                if(nbr.f_value <= max_f_value){
                    focal_list.push(nbr);
                    added_to_focal_list.insert(nbr.node_id);
                }
            }
        }
    }

    double search_time_elapsed = std::chrono::duration<double>(std::chrono::high_resolution_clock::now() - start_time).count();
    printf("Experiment Search Time: %.6f\n", search_time_elapsed);


    if(!solution_found){
        printf("\n\nNO SOLUTION FOUND!!!\n\n");
    }

    if(open_set.empty()){
        printf("Open set exhausted!!\n");
        printf("Open set size: %ld\n", open_set.size());
        printf("Focal set size: %ld\n", focal_list.size());
    }

    printf("Total nodes fully expanded: %d\n", num_fully_expanded);
    printf("Total expansions skipped: %d\n", num_skipped);
    printf("Total generations skipped because of inferior cost: %d\n", METRICS.num_skipped_duplicate_node);
    printf("Total generations skipped for high lazy f value: %d\n", METRICS.num_skipped_high_lazy_f_value);
    printf("Total generations discarded for high f value: %d\n", num_discarded_high_f);
    printf("Total nodes generated: %d\n", num_generated);
    printf("Total lazy batches run: %d\n", num_lazy_batches_run);

    printf("Total neighbor expansion time: %.3f seconds\n", METRICS.neighbor_expansion_time);
    printf("Total lazy f_value time: %.3f seconds\n", METRICS.lazy_f_value_calculation_time);
    printf("Total TSP f_value time: %.3f seconds\n", METRICS.tsp_f_value_calculation_time);
    printf("Total domination check time: %.3f seconds\n", METRICS.domination_check_time);
    printf("Total adding LOS time: %.3f seconds\n", METRICS.adding_los_time);
    printf("Max node depth expanded: %d\n", max_node_depth_expanded);
    printf("Max number of cities mtsp: %d\n", METRICS.mtsp_max_cities);
    for(int i = 0; i < solution_paths.size(); i++){
        printf("Path %d length: %ld\n", i, solution_paths[i].size());
        printf("\t[");
        for(Position pos : solution_paths[i]){
            printf("(%d,%d) ", pos.x, pos.y);
        }
        printf("]\n");
    }
    
    auto end_time = std::chrono::high_resolution_clock::now();
    auto seconds_taken = std::chrono::duration<double>(end_time - start_time).count();
    printf("Total search time taken: %.3f seconds\n", seconds_taken);

    debug_file.close();

    return solution_paths;
}
