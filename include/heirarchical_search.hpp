#pragma once

#include <vector>
#include <boost/dynamic_bitset.hpp>

#include "shared.hpp"
#include "los.hpp"
#include "search.hpp"

boost::dynamic_bitset<> get_partition_responsibility(const Map& map, std::vector<std::vector<Position>> paths, int agent_idx, const boost::dynamic_bitset<>& seen, const Lookup& lookup){
    boost::dynamic_bitset<> responsibility(map.num_squares);
    responsibility.set();
    printf("Starting responsibility count: %ld\n", responsibility.count());

    // Remove all squares that have already been seen + all obstacles.
    for(int map_idx = 0; map_idx < map.num_squares; map_idx++){
        Position pos = map.get_pos_from_map_idx(map_idx);
        if(seen[map_idx] || map.check_obstacle(pos)){
            responsibility[map_idx] = 0;
        }
    }

    printf("After removing start seen and obstacles: %ld\n", responsibility.count());

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

    printf("After removing other agents' LOS: %ld\n", responsibility.count());

    return responsibility;
}

// TODO: Have separate solver config for multi-agent and single-agent searches.
// TODO: Figure out task partition.
// TODO: How tf do i do multi-robot partition.
std::vector<std::vector<Position>> run_heirarchical_search(int start_timestep, std::vector<Position> starts, std::vector<Task> incomplete_tasks, boost::dynamic_bitset<> start_seen, const Map& map, const ProblemInput& problem_input, const Lookup& lookup){
    // First run a normal search to get the initial partition.
    SolverConfig solver_config = {
        .heuristic_type = problem_input.heuristic_type,
        .focal_epsilon = problem_input.centralized_focal_epsilon,
        .focal_heuristic_weight = problem_input.centralized_focal_heuristic_weight,
        .focal_search_time_limit = problem_input.centralized_focal_search_time_limit
    };
    std::vector<std::vector<Position>> centralized_solution = run_search(start_timestep, starts, incomplete_tasks, start_seen, map, solver_config, lookup);
    if(!problem_input.run_decentralized_search){
        return centralized_solution;
    }

    // Figure out which agent has the highest makespan.
    std::priority_queue<std::pair<int, int>> agent_to_update; // (makespan, agent_idx)
    for(int agent_idx = 0; agent_idx < centralized_solution.size(); agent_idx++){
        agent_to_update.push({centralized_solution[agent_idx].size(), agent_idx});
    }

    while(!agent_to_update.empty()){
        auto [makespan, agent_idx] = agent_to_update.top();
        agent_to_update.pop();

        // Get partition responsibility for this agent.
        boost::dynamic_bitset<> responsibility = get_partition_responsibility(map, centralized_solution, agent_idx, start_seen, lookup);
        printf("Responsibility for agent %d: %ld\n", agent_idx, responsibility.count());

        // Run single-agent search for this agent with their responsibility.
        SolverConfig single_agent_solver_config = {
            .heuristic_type = problem_input.heuristic_type,
            .focal_epsilon = problem_input.decentralized_focal_epsilon,
            .focal_heuristic_weight = problem_input.decentralized_focal_heuristic_weight,
            .focal_search_time_limit = problem_input.decentralized_focal_search_time_limit
        };
        std::vector<Position> single_agent_start = {starts[agent_idx]};
        boost::dynamic_bitset<> single_agent_seen = responsibility.flip();
        Lookup single_agent_lookup;
        precompute_lookup(single_agent_lookup, map, problem_input.heuristic_type, single_agent_start);
        print_map_state(single_agent_lookup, map, single_agent_seen, single_agent_start);
        std::vector<std::vector<Position>> single_agent_solution = run_search(start_timestep, single_agent_start, incomplete_tasks, single_agent_seen, map, single_agent_solver_config, single_agent_lookup);

        // Update centralized solution.
        if(single_agent_solution[0].size() < centralized_solution[agent_idx].size()){
            centralized_solution[agent_idx] = single_agent_solution[0];
        }
    }

    return centralized_solution;
}
