#pragma once

#include <vector>
#include <boost/dynamic_bitset.hpp>

#include "shared.hpp"
#include "los.hpp"
#include "collisions.hpp"
#include "search.hpp"

struct Partition {
    boost::dynamic_bitset<> vision;
    std::vector<Task> tasks;
};

boost::dynamic_bitset<> get_vision_partition_responsibility(const Map& map, std::vector<std::vector<Position>> paths, int agent_idx, const boost::dynamic_bitset<>& seen, const Lookup& lookup){
    boost::dynamic_bitset<> responsibility(map.num_squares);
    responsibility.set();

    // Remove all squares that have already been seen + all obstacles.
    for(int map_idx = 0; map_idx < map.num_squares; map_idx++){
        Position pos = map.get_pos_from_map_idx(map_idx);
        if(seen[map_idx] || map.check_obstacle(pos)){
            responsibility[map_idx] = 0;
        }
    }

    // Remove all squares that other agents' paths see.
    for(int other_agent_idx = 0; other_agent_idx < paths.size(); other_agent_idx++){
        if(other_agent_idx == agent_idx){
            continue;
        }
        for(Position pos : paths[other_agent_idx]){
            int map_idx = map.get_map_idx(pos);
            for(Position visible : lookup.los[map_idx]){
                int visible_map_idx = map.get_map_idx(visible);
                responsibility[visible_map_idx] = 0;
            }
        }
    }

    return responsibility;
}

std::vector<Task> get_task_partition_responsibility(int start_time, std::vector<std::vector<Position>> paths, int agent_idx, const std::vector<Task>& tasks_left){
    std::vector<Task> responsible_tasks;
    for(const Task& task : tasks_left){
        std::unordered_map<int, int> task_count;
        for(int other_agent_idx = 0; other_agent_idx < paths.size(); other_agent_idx++){
            if(other_agent_idx == agent_idx){
                continue;
            }
            for(int t = 0; t < paths[other_agent_idx].size(); t++){
                Position pos = paths[other_agent_idx][t];
                if(pos.equals(task.pos)){
                    task_count[t] += 1;
                }
            }
        }

        bool is_responsible = true;
        for(const auto& [time, count] : task_count){
            if(count >= task.num_agents_required){
                is_responsible = false;
                break;
            }
        }
        if(is_responsible){
            if(task.num_agents_required > 1){
                // find time range during which we can visit the task.
                int joint_finish_time = -1;
                for(int t = 0; t < paths[agent_idx].size(); t++){
                    Position pos = paths[agent_idx][t];
                    if(pos.equals(task.pos)){
                        if(task_count[t] + 1 >= task.num_agents_required && joint_finish_time == -1){
                            joint_finish_time = t;
                        }
                    }
                }

                // Find time range during which we can complete the task.
                int earliest_finish_time = joint_finish_time;
                while(task_count.find(earliest_finish_time) != task_count.end() && task_count[earliest_finish_time] >= task.num_agents_required - 1){
                    earliest_finish_time -= 1;
                }
                earliest_finish_time += 1;

                earliest_finish_time += start_time;
                joint_finish_time += start_time;
                
                Task task_copy = task;
                task_copy.num_agents_required = 1;
                task_copy.release_time = earliest_finish_time;
                task_copy.deadline = joint_finish_time;

                responsible_tasks.push_back(task_copy);

            } else {
                responsible_tasks.push_back(task);
            }
        }
    }
    return responsible_tasks;
}

bool is_task_subset(const std::vector<Task>& subset, const std::vector<Task>& superset){
    std::unordered_set<int> superset_task_ids;
    for(const Task& task : superset){
        superset_task_ids.insert(task.id);
    }
    for(const Task& task : subset){
        if(superset_task_ids.find(task.id) == superset_task_ids.end()){
            return false;
        }
    }
    return true;
}


std::string bitset_to_string(const boost::dynamic_bitset<>& bs) {
    std::string str;
    for (size_t i = 0; i < bs.size(); ++i) {
        str += bs[i] ? '1' : '0';
    }
    return str;
}

void add_decentralized_solution_to_history(std::unordered_map<std::string, std::vector<PastSolution>>& solution_history, std::vector<Position> solution, boost::dynamic_bitset<> seen, std::vector<Task> tasks_left, const Map& map, const Lookup& lookup, int start_time) {
    add_dominance_and_task_visibility_to_seen(seen, tasks_left, map, lookup);
    for(int i = 0; i < solution.size(); i++){
        add_los_to_seen(seen, lookup.los[map.get_map_idx(solution[i])], map);

        // Remove tasks that are completed.
        std::vector<Task> remaining_tasks;
        for(const Task& task : tasks_left){
            bool completed = false;
            if(solution[i].equals(task.pos) && start_time + i >= task.release_time && start_time + i <= task.deadline){
                completed = true;
            }
            if(!completed){
                remaining_tasks.push_back(task);
            }
        }
        tasks_left = remaining_tasks;

        AgentState curr_agent_state(solution[i], false, -1, start_time + i);
        std::string key = agent_states_to_string({curr_agent_state});
        if(solution_history.find(key) == solution_history.end()){
            solution_history[key] = std::vector<PastSolution>();
        }
        std::vector<Position> remaining_solution(solution.begin() + i, solution.end());
        solution_history[key].push_back({.path = remaining_solution, .seen = seen, .tasks_left = tasks_left});
    }
}


std::vector<std::vector<Position>> run_heirarchical_search(int start_timestep, std::vector<Position> starts, std::vector<Task> incomplete_tasks, boost::dynamic_bitset<> start_seen, const Map& map, const ProblemInput& problem_input, const Lookup& lookup, std::unordered_map<std::string, std::vector<PastSolution>>& solution_history, MetricsList& aggregated){
    printf("\n\n\n\n");
    // First run a normal search to get the initial partition.
    SolverConfig solver_config = {
        .heuristic_type = problem_input.heuristic_type,
        .focal_method = problem_input.focal_method,
        .optimizations = problem_input.optimizations,
        .focal_epsilon = problem_input.centralized_focal_epsilon,
        .focal_heuristic_weight = problem_input.centralized_focal_heuristic_weight,
        .focal_search_time_limit = problem_input.centralized_focal_search_time_limit
    };
    auto start_time = std::chrono::high_resolution_clock::now();
    A_STAR_WEIGHT = problem_input.centralized_astar_weight;
    std::vector<std::vector<Position>> multi_agent_solution = run_search(start_timestep, starts, incomplete_tasks, start_seen, map, solver_config, lookup, {});
    aggregated.add_metrics(METRICS);
    auto end_time = std::chrono::high_resolution_clock::now();
    std::chrono::duration<double> duration = end_time - start_time;
    printf("Centralized search time: %.6f seconds\n", duration.count());
    aggregated.centralized_search_time.push_back(duration.count());
    if(!problem_input.run_decentralized_search){
        add_waits_to_end(multi_agent_solution);
        return multi_agent_solution;
    }

    // Figure out which agent has the highest makespan.
    std::vector<bool> should_retry = std::vector<bool>(starts.size(), true);
    std::vector<std::vector<Partition>> partitions_solved_by_agent(starts.size(), std::vector<Partition>());

    printf("\nStarting decentralized search\n");
    int num_decentralized_searches = 0;
    while(num_decentralized_searches < problem_input.max_decentralized_searches){
        start_time = std::chrono::high_resolution_clock::now();
        if(num_decentralized_searches > 0){
            printf("\n");
        }
        int best_agent_idx = -1;
        int largest_agent_makespan = 0;
        for(int agent_idx = 0; agent_idx < multi_agent_solution.size(); agent_idx++){
            if(should_retry[agent_idx]){
                int agent_makespan = multi_agent_solution[agent_idx].size();
                if(agent_makespan > largest_agent_makespan){
                    largest_agent_makespan = agent_makespan;
                    best_agent_idx = agent_idx;
                }
            }
        }

        if(best_agent_idx == -1){
            // No more agents to retry.
            break;
        }

        // Get partition responsibility for this agent.
        int agent_idx = best_agent_idx;
        Partition responsibility = {
            .vision = get_vision_partition_responsibility(map, multi_agent_solution, agent_idx, start_seen, lookup),
            .tasks = get_task_partition_responsibility(start_timestep, multi_agent_solution, agent_idx, incomplete_tasks)
        };

        printf("Retrying agent %d with current path length %d\n", best_agent_idx, largest_agent_makespan);
        printf("\tVision responsibility num squares: %ld\n", responsibility.vision.count());
        printf("\tTask responsibility num tasks: %ld\n", responsibility.tasks.size());

        // Check if we've already solved this partition for this agent.
        bool already_solved = false;
        for(const auto& prev_partition : partitions_solved_by_agent[agent_idx]){
            if(responsibility.vision.is_subset_of(prev_partition.vision) && is_task_subset(responsibility.tasks, prev_partition.tasks)){
                printf("Already solved this partition for agent %d, skipping...\n", agent_idx);
                already_solved = true;
                break;
            }
        }
        if(already_solved){
            should_retry[agent_idx] = false;
            continue;
        }

        // Run single-agent search for this agent with their responsibility.
        SolverConfig single_agent_solver_config = {
            .heuristic_type = problem_input.heuristic_type,
            .focal_method = problem_input.focal_method,
            .optimizations = problem_input.optimizations,
            .focal_epsilon = problem_input.decentralized_focal_epsilon,
            .focal_heuristic_weight = problem_input.decentralized_focal_heuristic_weight,
            .focal_search_time_limit = problem_input.decentralized_focal_search_time_limit
        };
        std::vector<Position> single_agent_start = {starts[agent_idx]};
        boost::dynamic_bitset<> single_agent_seen = ~responsibility.vision;
        Lookup single_agent_lookup = lookup;
        single_agent_lookup.strictly_easier = lookup.strictly_easier_per_agent[agent_idx];
        printf("Map state before decentralized search: %s\n", get_map_state(single_agent_lookup, map, single_agent_seen, single_agent_start).c_str());

        end_time = std::chrono::high_resolution_clock::now();
        duration = end_time - start_time;
        printf("Time taken before search calculation: %.6f seconds\n", duration.count());

        A_STAR_WEIGHT = problem_input.decentralized_astar_weight;
        std::vector<std::vector<Position>> single_agent_solution = run_search(start_timestep, single_agent_start, responsibility.tasks, single_agent_seen, map, single_agent_solver_config, single_agent_lookup, solution_history);
        aggregated.add_metrics(METRICS);
        add_decentralized_solution_to_history(solution_history, single_agent_solution[0], single_agent_seen, responsibility.tasks, map, single_agent_lookup, start_timestep);

        // Update centralized solution.
        if(single_agent_solution[0].size() < multi_agent_solution[agent_idx].size()){
            multi_agent_solution[agent_idx] = single_agent_solution[0];

            // Now that we've updated this agent's path, all other agents need to retry since their partitions may have changed.
            for(int other_agent_idx = 0; other_agent_idx < multi_agent_solution.size(); other_agent_idx++){
                if(other_agent_idx != agent_idx){
                    should_retry[other_agent_idx] = true;
                }
            }
        }
        partitions_solved_by_agent[agent_idx].push_back(responsibility);
        should_retry[agent_idx] = false;
        num_decentralized_searches += 1;
        end_time = std::chrono::high_resolution_clock::now();
        duration = end_time - start_time;
        printf("Time taken after search: %.6f seconds\n", duration.count());
        aggregated.decentralized_search_time.push_back(duration.count());
    }

    aggregated.num_decentralized_searches.push_back(num_decentralized_searches);

    add_waits_to_end(multi_agent_solution);
    return multi_agent_solution;
}
