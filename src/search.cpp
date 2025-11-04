#include "search.hpp"
#include "shared.hpp"
#include "utils.hpp"
#include "pathfinding.hpp"
#include "heuristics.hpp"
#include "collisions.hpp"
#include "los.hpp"
#include <unordered_map>
#include <iostream>
#include <chrono>
#include <boost/functional/hash.hpp> // For boost::hash_value
#include <boost/heap/fibonacci_heap.hpp>
#include "BS_thread_pool.hpp"

std::vector<std::pair<int, int>> get_f_and_focal_values(HeuristicType heuristic_type, const Map& map, const std::vector<HeuristicInput>& neighbor_heuristic_inputs, double focal_heuristic_weight, const Lookup& lookup) {
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
            // DisjointGraph disjoint_graph = compute_disjoint_graph(lookup, non_terminated_agent_map_idxs, input.seen, input.tasks_left);
            DisjointGraph disjoint_graph = compute_disjoint_graph(map, non_terminated_agents, input.seen, input.tasks_left, lookup);
            prune_graph(disjoint_graph, lookup);
            alter_disjoint_graph_for_waiting_robots(disjoint_graph, map, non_terminated_agents, input.tasks_left, lookup);

            // If there's no pivots, just return the singleton heuristic.
            if(disjoint_graph.pivots.size() == 0){
                use_singleton = true;
            } else {
                std::vector<int> non_terminated_agent_costs;
                for(const auto& agent : non_terminated_agents){
                    non_terminated_agent_costs.push_back(agent.cost);
                }
                tsp_idxs.push_back(i);
                futures.push_back(pool.submit_task([disjoint_graph, non_terminated_agent_costs]() {
                    return get_multi_tsp_f_and_focal_value(disjoint_graph, non_terminated_agent_costs);
                }));
            }
        }

        // Singleton heuristic.
        if(use_singleton) {
            f_and_focal_values[i].first = std::max(f_and_focal_values[i].first, get_singleton_f_value(input.agents, map, input.cost, input.seen, input.tasks_left, lookup));
            // Make copy of agents, but at 0 cost. Focal value is the singleton heuristic if the search just started (estimate of how much searching there is left to do).
            std::vector<AgentState> agents_copy = input.agents;
            for(auto& agent : agents_copy){
                agent.cost = 0;
            }
            f_and_focal_values[i].second = get_singleton_f_value(agents_copy, map, 0, input.seen, input.tasks_left, lookup);
        }
    }

    // Collect TSP/MAX heuristic results.
    for(int i = 0; i < tsp_idxs.size(); i++) {
        auto result = futures[i].get();
        f_and_focal_values[tsp_idxs[i]].first = std::max(f_and_focal_values[tsp_idxs[i]].first, result.first); // Ensure that f value is max of singleton and TSP.
        f_and_focal_values[tsp_idxs[i]].second = std::max(f_and_focal_values[tsp_idxs[i]].second, result.second); // Ensure that focal value is max of singleton and TSP.
    }

    // Weighted A* for focal heuristic, but with sum of costs instead of makespan.
    for(int i = 0; i < f_and_focal_values.size(); i++) {
        double current_sum = 0.0;
        for(const auto& agent : neighbor_heuristic_inputs[i].agents){
            current_sum += agent.cost;
        }
        f_and_focal_values[i].second = current_sum + focal_heuristic_weight * f_and_focal_values[i].second;
    }

    return f_and_focal_values;
}

int get_wait_action_end_time(const Map& map, const Lookup& lookup, const std::vector<AgentState>& agents, Task task, int agent_cost){
    // If we're at a task, clearly it hasn't been completed yet, so I guess more robots need to come. Add a wait action to wait until however long it'll take someone else to come.
    std::vector<int> agent_min_arrival_times;
    for(AgentState agent : agents) {
        // Ignore agents that are terminated or waiting at other tasks, since they can't really come help right now.
        if(agent.terminated || (agent.waiting_idx != -1 && agent.waiting_idx != task.id)) {
            continue;
        }
        int agent_map_idx = map.get_map_idx(agent.pos);
        agent_min_arrival_times.push_back(agent.cost + lookup.apsp[agent_map_idx][task.map_idx]);
    }

    std::sort(agent_min_arrival_times.begin(), agent_min_arrival_times.end());
    // Since we need num_agents_required agents to reach the task. NOTE: It is possible that there aren't num_agents_required available, because other agents are waiting, and will be freed up once their task is completed.
    int min_alt_wait_time = agent_min_arrival_times[std::min(task.num_agents_required - 1, (int)agent_min_arrival_times.size() - 1)];
    return std::max(agent_cost, min_alt_wait_time);
}


std::vector<std::vector<AgentState>> get_possible_moves(const Map& map, const std::vector<AgentState>& agents, const boost::dynamic_bitset<>& seen, const std::vector<Task>& tasks_left, const Lookup& lookup, int agent_to_expand){
    std::vector<std::vector<AgentState>> options;
    for(AgentState agent : agents){
        if(agent.terminated){
            // If any agent is terminated, we don't generate any neighbors.
            options.push_back({agent});
            continue;
        }
        std::vector<AgentState> agent_options;

        int at_incomplete_task_idx = -1;
        for(const Task& t : tasks_left){
            if(map.get_map_idx(agent.pos) == t.map_idx){
                at_incomplete_task_idx = t.id;
                break;
            }
        }

        // If not waiting, but we're at a task, and the task is incomplete, then we can wait at the task.
        // If we're currently waiting, we should continue to wait at the task.
        if(agent.waiting_idx != -1 || at_incomplete_task_idx != -1){
            // printf("WE'RE AT AN INCOMPLETE TASK, ADDING WAIT OPTION\n");
            // Figure out how long to wait for.
            int wait_task_idx = (agent.waiting_idx != -1) ? agent.waiting_idx : at_incomplete_task_idx;
            Task task = get_task_by_id(tasks_left, wait_task_idx);
            int wait_time = get_wait_action_end_time(map, lookup, agents, task, agent.cost);
            agent_options.push_back(AgentState(agent.pos, false, wait_task_idx, wait_time));

            // If we're currently waiting, we can't move until we complete the task.
            if(agent.waiting_idx != -1){
                options.push_back(agent_options);
                continue;
            }
        }

        // Expanding borders implementation.
        std::vector<std::tuple<Position, int>> nbrs_with_added_cost = get_extended_neighbors(map, agent.pos, seen, tasks_left, lookup);
        for(auto [nbr, added_cost] : nbrs_with_added_cost){
            agent_options.push_back(AgentState(nbr, false, -1, agent.cost + added_cost));
        }

        agent_options.push_back(AgentState(agent.pos, true, -1, agent.cost)); // Option to terminate.
        options.push_back(agent_options);
    }

    // Now, take the sum of the options. Don't include states in which all of the agents terminate.
    std::vector<std::vector<AgentState>> all_moves;
    // for(int i = 0; i < agents.size(); i++){

    // agent_to_expand = -1;
    // for(int i = 0; i < agents.size(); i++){
    //     if(agents[i].terminated){
    //         continue;
    //     }
    //     if(agent_to_expand == -1 || agents[i].cost < agents[agent_to_expand].cost){
    //         agent_to_expand = i;
    //     }
    // }

    // for(int i = agent_to_expand; i <= agent_to_expand; i++){
    //     for(AgentState option : options[i]){
    //         std::vector<AgentState> move = agents;
    //         move[i] = option;
    //         bool skip = true;
    //         for(AgentState a : move){
    //             if(!a.terminated){
    //                 skip = false;
    //                 break;
    //             }
    //         }
    //         if(skip){
    //             continue;
    //         }
    //         all_moves.push_back(move);
    //     }
    // }

    // An agent is considered 'free' if it is not terminated and not waiting at a task.
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

std::vector<Node> get_neighbors(Node& node, const Map& map, const Lookup& lookup, SolverConfig solver_config, double best_solution_cost, int last_id_assigned, std::unordered_map<std::string, std::vector<VisitedNodeInfo>>& generated_costs, std::unordered_set<int>& avoid_expansion_list){
    int agent_to_expand = (node.last_agent_expanded + 1) % node.agents.size();
    while(node.agents[agent_to_expand].terminated){
        agent_to_expand = (agent_to_expand + 1) % node.agents.size();
    }

    std::vector<Node> neighbor_nodes;
    auto start = std::chrono::high_resolution_clock::now();
    auto neighbors = get_possible_moves(map, node.agents, node.seen, node.tasks_left, lookup, agent_to_expand);
    auto end = std::chrono::high_resolution_clock::now();
    std::chrono::duration<double> duration = end - start;
    METRICS.neighbor_expansion_time += duration.count();

    std::vector<HeuristicInput> neighbor_heuristic_inputs;

    for(auto& nbr : neighbors){
        boost::dynamic_bitset<> nbr_seen = node.seen;
        // For each neighbor, loop through path to neighbor.
        int new_squares_seen = 0;
        int nbr_cost = 0;
        for(const AgentState& agent : nbr){
            new_squares_seen += add_los_to_seen(nbr_seen, lookup.los[map.get_map_idx(agent.pos)], map);
            nbr_cost = std::max(nbr_cost, agent.cost);

            // If the agent has reached the task, remove it from the tasks left.
            int agent_map_idx = map.get_map_idx(agent.pos);
        }

        std::vector<Task> nbr_tasks_left;
        bool task_failed = false;
        for(Task t : node.tasks_left){
            int task_map_idx = map.get_map_idx(t.pos);
            std::unordered_map<int, int> num_reached_by_time;
            bool reachable = false;
            bool completed = false;
            for(const AgentState& agent : nbr){
                int agent_map_idx = map.get_map_idx(agent.pos);
                if(agent_map_idx == task_map_idx){
                    num_reached_by_time[agent.cost] += 1;
                    if(num_reached_by_time[agent.cost] >= t.num_agents_required){
                        completed = true;
                        break;
                    }
                }
            }
            if(completed) {
                // Free up agents that were waiting at this task.
                for(AgentState& agent : nbr){
                    if(agent.waiting_idx == t.id){
                        agent.waiting_idx = -1;
                    }
                }
            }
            else {
                nbr_tasks_left.push_back(t);
            }
        }

        // Check if we have a deadlock. That is, check if there is at least one task that is reachable by at least one non-terminated agent.
        bool task_deadlocked = false;
        for(const Task& t : nbr_tasks_left){
            int agents_available = 0;
            for(const AgentState& agent : nbr){
                if(agent.waiting_idx == t.id || (!agent.terminated && agent.waiting_idx == -1)){
                    agents_available += 1;
                    continue;
                }
            }
            if(agents_available < t.num_agents_required){
                task_deadlocked = true;
                break;
            }
        }

        if(task_deadlocked) {
            // Don't generate neighbors that have deadlocked tasks.
            // printf("Deadlocked task detected, skipping neighbor generation.\n");
            METRICS.num_skipped_task_deadlock += 1;
            continue;
        }

        int nbr_num_seen = node.num_seen + new_squares_seen;

        // TODO: Have to change this if we add in tasks that take a certain amount of time to complete (instead of just task id should also be time remaining on task).
        std::string nbr_key = agent_states_to_string(nbr) + task_array_hash_string(nbr_tasks_left);
        std::vector<AgentState> nbr_sorted = get_sorted_agents_by_position(nbr);

        // Check for dominance in generated costs.
        start = std::chrono::high_resolution_clock::now();
        bool dominated = false;
        // Current node is dominated by existing node, skip.
        if(generated_costs.find(nbr_key) != generated_costs.end()){
            auto& visited_node_list = generated_costs[nbr_key];
            auto it = visited_node_list.begin();
            while (it != visited_node_list.end()) {
                bool nbr_cost_better = true;
                bool visited_cost_better = true;
                for(int i = 0; i < nbr_sorted.size(); i++){
                    if(it->agents[i].cost < nbr_sorted[i].cost){
                        nbr_cost_better = false;
                    } else if(nbr_sorted[i].cost < it->agents[i].cost){
                        visited_cost_better = false;
                    }
                }

                if(visited_cost_better && nbr_seen.is_subset_of(it->seen)){
                    // Existing node dominates current node, skip current node.
                    METRICS.num_skipped_duplicate_node += 1;
                    dominated = true;
                    break;
                } else if(nbr_cost_better && it->seen.is_subset_of(nbr_seen)){
                    // Current node dominates existing node, remove existing node.
                    it = visited_node_list.erase(it);
                    avoid_expansion_list.insert(it->id);
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

        double min_nbr_f_value = get_singleton_f_value(nbr, map, nbr_cost, nbr_seen, nbr_tasks_left, lookup);
        if(min_nbr_f_value >= best_solution_cost){
            // Don't generate neighbors that are already worse than the best solution found so far.
            METRICS.num_skipped_high_lazy_f_value += 1;
            continue;
        }

        neighbor_heuristic_inputs.push_back(HeuristicInput{nbr, nbr_cost, nbr_seen, nbr_tasks_left, nbr_num_seen});
    }

    HeuristicType heuristic_type = solver_config.heuristic_type;
    if(heuristic_type == LAZY){
        heuristic_type = SINGLETON;
    }

    start = std::chrono::high_resolution_clock::now();
    std::vector<std::pair<int, int>> f_and_focal_values = get_f_and_focal_values(heuristic_type, map, neighbor_heuristic_inputs, solver_config.focal_heuristic_weight, lookup);
    end = std::chrono::high_resolution_clock::now();
    duration = end - start;
    METRICS.f_value_calculation_time += duration.count();

    for(int i = 0; i < neighbor_heuristic_inputs.size(); i++){
        // Apply pathmax to ensure consistency.
        int nbr_f_value = std::max(f_and_focal_values[i].first, node.f_value);
        int nbr_focal_value = f_and_focal_values[i].second;
        last_id_assigned += 1;
        const HeuristicInput& input = neighbor_heuristic_inputs[i];

        std::string nbr_key = agent_states_to_string(input.agents) + task_array_hash_string(input.tasks_left);
        std::vector<AgentState> nbr_sorted = get_sorted_agents_by_position(input.agents);
        if(generated_costs.find(nbr_key) == generated_costs.end()){
            generated_costs[nbr_key] = std::vector<VisitedNodeInfo>{};
        }
        generated_costs[nbr_key].push_back(VisitedNodeInfo{last_id_assigned, nbr_sorted, input.seen});

        neighbor_nodes.push_back(Node(last_id_assigned, input.agents, input.seen, input.tasks_left, input.cost, nbr_f_value, nbr_focal_value, input.num_seen, agent_to_expand, node.depth + 1));
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

// Inputs: Agent starting positions, Tasks, LOS type, map.
// Output: Optimal path.
std::vector<std::vector<Position>> run_search(int start_timestep, std::vector<Position> starts, std::vector<Task> incomplete_tasks, boost::dynamic_bitset<> start_seen, const Map& map, const SolverConfig& solver_config, const Lookup& lookup){
    // Reset metrics for every search run.
    METRICS.reset();

    // Create initial seen bitset.
    int num_start_seen = start_seen.count();
    for(int map_idx = 0; map_idx < map.num_squares; map_idx++){
        if(start_seen[map_idx]){
            continue;
        }
        // Mark squares that are within los of a task as "seen" since they will be explored when completing the task.
        // Don't include squares that are the task themselves.
        bool is_within_los_of_task = false;
        for(const Task& task : incomplete_tasks){
            if(map_idx == task.map_idx){
                is_within_los_of_task = false;
                continue;
            }
            if(lookup.watchers_set[map_idx].find(task.map_idx) != lookup.watchers_set[map_idx].end()){
                is_within_los_of_task = true;
            }
        }
        if(lookup.strictly_easier[map_idx] || map.check_obstacle(map.get_pos_from_map_idx(map_idx)) || is_within_los_of_task){
            start_seen[map_idx] = 1;
            num_start_seen += 1;
        }
    }
    std::vector<AgentState> start_agent_states;
    for(Position start : starts){
        num_start_seen += add_los_to_seen(start_seen, lookup.los[map.get_map_idx(start)], map);
        start_agent_states.push_back(AgentState(start, false, -1, start_timestep));
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
    printf("Incomplete tasks (%ld): \n", incomplete_tasks.size());
    for(const Task& task : incomplete_tasks){
        printf("\t%s\n", task.toString().c_str());
        if(task.num_agents_required > starts.size()){
            printf("Task %s requires more agents (%d) than are available (%ld). No solution possible.\n", task.toString().c_str(), task.num_agents_required, starts.size());
            exit(1);
        }
    }
    auto [start_f_value, start_focal_value] = get_f_and_focal_values(start_heuristic_type, map, {HeuristicInput{start_agent_states, start_timestep, start_seen, incomplete_tasks, num_start_seen}}, solver_config.focal_heuristic_weight, lookup)[0];
    printf("Start f value: %d, Start focal value: %d, Num start seen: %d / %d\n", start_f_value, start_focal_value, num_start_seen, map.num_squares);
    // exit(0);

    handle_lookup[0] = open_set.push(Node(/* id = */ 0, start_agent_states, start_seen, incomplete_tasks, /* cost = */ start_timestep, start_f_value, start_focal_value, num_start_seen, /*last_agent_expanded = */ 0, /*depth = */ 0));

    std::vector<Node> expanded_nodes;

    int num_expanded = 0;
    int num_generated = 0;
    int last_id_assigned = 0;
    int max_new_squares_seen = 0;
    int num_skipped = 0;
    int num_skipped_dom = 0;
    int num_fully_expanded = 0;
    int num_discarded_high_f = 0;

    auto start_time = std::chrono::high_resolution_clock::now();

    std::vector<std::vector<Position>> solution_paths(starts.size(), std::vector<Position>());
    bool solution_found = false;
    int max_node_depth_expanded = 0;
    double best_solution_cost = std::numeric_limits<double>::infinity();

    while(!open_set.empty()){
        if(solution_found && (std::chrono::duration<double>(std::chrono::high_resolution_clock::now() - start_time).count() > solver_config.focal_search_time_limit)){
            printf("Search time limit reached, ending search.\n");
            break;
        }

        // Check if focal list needs to be updated.
        int min_f_value = open_set.top().f_value;
        if(min_f_value > prev_min_f){
            prev_min_f = min_f_value;
            // Add to focal list.
            auto it = open_set.ordered_begin();
            int max_f_value = (int)std::ceil(solver_config.focal_epsilon * (double)min_f_value);
            while(it != open_set.ordered_end() && it->f_value <= max_f_value){
                if(added_to_focal_list.find(it->node_id) == added_to_focal_list.end()){
                    focal_list.push(*it);
                    added_to_focal_list.insert(it->node_id);
                }
                ++it;
            }
        }

        // Get lowest cost node by focal heuristic.
        Node curr = focal_list.top();
        focal_list.pop();
        // Remove from open set as well.
        open_set.erase(handle_lookup[curr.node_id]);

        // Skip expansion if we've found a better solution.
        if(curr.f_value >= best_solution_cost){
            continue;
        }

        if(avoid_expansion_list.find(curr.node_id) != avoid_expansion_list.end()){
            // This node has been dominated by a node generated after it, so skip its expansion.
            num_skipped += 1;
            continue;
        }

        num_expanded += 1;

        if(solver_config.heuristic_type == LAZY && curr.is_lazy){
            // Recompute f value.
            auto [new_f_value, new_focal_value] = get_f_and_focal_values(HeuristicType::TSP, map, {HeuristicInput{curr.agents, curr.cost, curr.seen, curr.tasks_left, curr.num_seen}}, solver_config.focal_heuristic_weight, lookup)[0];
            // int new_f_value = get_f_value(HeuristicType::TSP, map, curr.agents, curr.cost, curr.seen, curr.tasks_left, lookup);
            new_f_value = std::max(new_f_value, curr.f_value); // Ensure f value never decreases.
            new_focal_value = std::max(new_focal_value, curr.focal_heuristic); // Ensure focal value never decreases.
            curr.update_f_value(new_f_value);
            curr.update_focal_heuristic(new_focal_value);
            open_set.update(handle_lookup[curr.node_id], curr);
            continue;
        }

        num_fully_expanded += 1;

        expanded_nodes.push_back(curr);
        max_node_depth_expanded = std::max(max_node_depth_expanded, curr.depth);

        id_lookup[curr.node_id] = curr.agents;

        write_node_to_file(debug_file, curr, lookup, map, pred_lookup[curr.node_id], solver_config.heuristic_type);

        max_new_squares_seen = std::max(max_new_squares_seen, curr.num_seen - num_obstacles);
        if(num_fully_expanded % 100 == 0){
            printf("Expanded %d nodes. Fully expanded %d nodes. Num generated %d. Loc: %s, cost: %d, heuristic: %d, num free seen: %d / %d, max free squares seen: %d\n", num_expanded, num_fully_expanded, num_generated, agent_states_to_print_string(curr.agents).c_str(), curr.cost, curr.heuristic, (curr.num_seen - num_obstacles), num_free, max_new_squares_seen);
            printf("\tF value: %d. Cost: %d. Heuristic: %d. Focal: %d\n", curr.f_value, curr.cost, curr.heuristic, curr.focal_heuristic);
            printf("\tNode depth: %d, Max node depth expanded: %d. Min f value: %d, Max f value searching: %d\n", curr.depth, max_node_depth_expanded, prev_min_f, (int)(solver_config.focal_epsilon * prev_min_f));
        }

        // Goal condition.
        if(curr.num_seen == map.num_squares && curr.tasks_left.size() == 0){
            printf("Goal condition met!\n");
            printf("Num seen: %d / %d\n", curr.num_seen, map.num_squares);
            printf("Solution node depth: %d\n", curr.depth);
            printf("Solution cost: %d\n", curr.cost);
            for(AgentState agent : curr.agents){
                printf("\tAgent final time: %d\n", agent.cost);
            }

            solution_found = true;
            best_solution_cost = curr.cost;
            solution_paths = reconstruct_path(curr.node_id, pred_lookup, id_lookup, starts, map, lookup);
            // for(const auto& path : solution_paths){
            //     printf("Path length: %ld\n", path.size());
            // }
            continue;
            // break;
        }

        std::vector<Node> neighbors = get_neighbors(curr, map, lookup, solver_config, best_solution_cost, last_id_assigned, generated_costs, avoid_expansion_list);

        num_generated += neighbors.size();
        if(neighbors.size() > 0){
            last_id_assigned = neighbors.back().node_id;
        }
        for(Node& nbr : neighbors){
            if(nbr.f_value >= best_solution_cost){
                num_discarded_high_f += 1;
                continue;
            }

            // printf("\tGenerated neighbor. Node ID: %d, Loc: %s, cost: %d, heuristic: %d, num seen: %d\n", nbr.node_id, nbr.pos.toString().c_str(), nbr.cost, nbr.heuristic, nbr.num_seen);
            pred_lookup[nbr.node_id] = curr.node_id;
            handle_lookup[nbr.node_id] = open_set.push(nbr);
            if(nbr.f_value <= (int)(solver_config.focal_epsilon * prev_min_f)){
                focal_list.push(nbr);
                added_to_focal_list.insert(nbr.node_id);
            }
        }
    }

    if(!solution_found){
        printf("\n\nNO SOLUTION FOUND!!!\n\n");
    }

    if(open_set.empty()){
        printf("Open set exhausted!!\n");
    }

    // add_waits_to_end(solution_paths);

    // for(int i = 0; i < solution_paths.size(); i++){
    //     printf("Path for agent %d (length %ld): %s\n", i, solution_paths[i].size(), pos_array_to_string(solution_paths[i]).c_str());
    // }

    printf("Total nodes expanded: %d\n", num_expanded);
    printf("Total nodes fully expanded: %d\n", num_fully_expanded);
    printf("Total expansions skipped: %d\n", num_skipped);
    // printf("Total expansions skipped by domination check: %d\n", num_skipped_dom);
    // printf("Total generations skipped because of inferior cost: %d\n", METRICS.num_skipped_duplicate_node);
    // printf("Total generations skipped because of task deadlock: %d\n", METRICS.num_skipped_task_deadlock);
    // printf("Total generations skipped for high lazy f value: %d\n", METRICS.num_skipped_high_lazy_f_value);
    // printf("Total generations discarded for high f value: %d\n", num_discarded_high_f);
    printf("Total nodes generated: %d\n", num_generated);
    // if(solver_config.heuristic_type == TSP || solver_config.heuristic_type == MAX || solver_config.heuristic_type == LAZY){
    //     printf("MTSP Setup time: %.3f seconds\n", METRICS.mtsp_setup_time);
    //     printf("MTSP Solver time: %.3f seconds\n", METRICS.mtsp_solver_runtime);
    //     printf("Total MTSP calls: %d\n", METRICS.mtsp_total_calls);
    // }
    printf("Total neighbor expansion time: %.3f seconds\n", METRICS.neighbor_expansion_time);
    printf("Total get_f_value time: %.3f seconds\n", METRICS.f_value_calculation_time);
    printf("Total domination check time: %.3f seconds\n", METRICS.domination_check_time);
    printf("Max node depth expanded: %d\n", max_node_depth_expanded);
    for(int i = 0; i < solution_paths.size(); i++){
        printf("Path %d length: %ld\n", i, solution_paths[i].size());
    }
    
    auto end_time = std::chrono::high_resolution_clock::now();
    auto seconds_taken = std::chrono::duration<double>(end_time - start_time).count();
    printf("Total search time taken: %.3f seconds\n", seconds_taken);

    debug_file.close();

    std::ofstream solution_file;
    solution_file.open("search_solution.csv");
    solution_file << "Timestep, Num Agents, Num seen, Agent positions, Seen Bitset\n"; // Header
    solution_file << map.map_name << "\n";
    write_solution_to_file(solution_file, solution_paths, start_seen, map, lookup);
    solution_file.close();

    return solution_paths;
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