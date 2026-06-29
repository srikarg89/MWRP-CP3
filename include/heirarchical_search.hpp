#pragma once

#include <vector>
#include <boost/dynamic_bitset.hpp>

#include "shared.hpp"
#include "los.hpp"
#include "search.hpp"

struct Partition {
    boost::dynamic_bitset<> vision;
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

std::vector<std::vector<Position>> run_heirarchical_search(int start_timestep, std::vector<Position> starts, boost::dynamic_bitset<> start_seen, const Map& map, const ProblemInput& problem_input, const Lookup& lookup, MetricsList& aggregated){
    printf("\n");
    // First run a normal search to get the initial partition.
    SolverConfig solver_config = {
        .heuristic_type = problem_input.heuristic_type,
        .focal_method = problem_input.focal_method,
        .optimizations = problem_input.optimizations,
        .focal_epsilon = problem_input.centralized_focal_epsilon,
        .focal_heuristic_weight = problem_input.centralized_focal_heuristic_weight,
        .search_time_limit = problem_input.centralized_search_time_limit,
        .hard_search_time_limit = problem_input.centralized_hard_search_time_limit
    };
    auto start_time = std::chrono::high_resolution_clock::now();
    A_STAR_WEIGHT = problem_input.centralized_astar_weight;
    std::vector<std::vector<Position>> multi_agent_solution = run_search(start_timestep, starts, start_seen, map, solver_config, lookup);
    aggregated.add_metrics(METRICS);
    auto end_time = std::chrono::high_resolution_clock::now();
    std::chrono::duration<double> duration = end_time - start_time;
    printf("Centralized search time: %.6f seconds\n", duration.count());
    aggregated.centralized_search_time.push_back(duration.count());
    if(!problem_input.run_decentralized_search){
        add_waits_to_end(multi_agent_solution);
        return multi_agent_solution;
    }

    int centralized_cost = 0;
    for(const auto& path : multi_agent_solution){
        if(path.size() > centralized_cost){
            centralized_cost = path.size();
        }
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
        };

        printf("Retrying agent %d with current path length %d\n", best_agent_idx, largest_agent_makespan);
        printf("\tVision responsibility num squares: %ld\n", responsibility.vision.count());

        // Check if we've already solved this partition for this agent.
        bool already_solved = false;
        for(const auto& prev_partition : partitions_solved_by_agent[agent_idx]){
            if(responsibility.vision.is_subset_of(prev_partition.vision)){
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
            .search_time_limit = problem_input.decentralized_search_time_limit,
            .hard_search_time_limit = problem_input.decentralized_hard_search_time_limit
        };
        std::vector<Position> single_agent_start = {starts[agent_idx]};
        boost::dynamic_bitset<> single_agent_seen = ~responsibility.vision;
        Lookup single_agent_lookup = lookup;
        std::vector<bool> agent_strictly_easier = lookup.strictly_easier;
        // Add any squares that are dominated by the agent's responsibility to the strictly easier list for this agent.
        if(lookup.per_agent_path_dominations.size() > agent_idx){
            for(int map_idx = 0; map_idx < map.num_squares; map_idx++){
                if(!agent_strictly_easier[map_idx]) {
                    // Check if this square is dominated by any of the agent's responsibility squares.
                    agent_strictly_easier[map_idx] = (lookup.per_agent_path_dominations[agent_idx][map_idx] & responsibility.vision).any();
                }
            }
        }
        single_agent_lookup.strictly_easier = agent_strictly_easier;
        printf("Map state before decentralized search: %s\n", get_map_state(single_agent_lookup, map, single_agent_seen, single_agent_start).c_str());

        end_time = std::chrono::high_resolution_clock::now();
        duration = end_time - start_time;
        printf("Time taken before search calculation: %.6f seconds\n", duration.count());

        A_STAR_WEIGHT = problem_input.decentralized_astar_weight;
        std::vector<std::vector<Position>> single_agent_solution = run_search(start_timestep, single_agent_start, single_agent_seen, map, single_agent_solver_config, single_agent_lookup);
        aggregated.add_metrics(METRICS);

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
    int decentralized_cost = multi_agent_solution[0].size();

    printf("\nFinal centralized cost: %d\n", centralized_cost);
    printf("Final decentralized cost: %d\n\n", decentralized_cost);

    return multi_agent_solution;
}
