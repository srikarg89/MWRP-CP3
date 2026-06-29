#include <iostream>
#include <chrono>
#include <algorithm>
#include <ctime>
#include "search.hpp"
#include "environment.hpp"
#include "heirarchical_search.hpp"

std::tuple<ScenarioConfig, ProblemInput> parse_arguments(int argc, char **argv) {
    // Setup scenario config.
    ScenarioConfig scenario_config = ScenarioConfig::from_json(argv[1]);
    ProblemInput problem_input = ProblemInput::from_json(argv[2]);
    printf("Scenario Config Loaded: %s\n", argv[1]);
    printf("Problem Input Loaded: %s\n", argv[2]);
    printf("Heuristic: %s\n", heuristic_type_to_string(problem_input.heuristic_type).c_str());
    printf("Focal Method: %s\n", focal_method_to_string(problem_input.focal_method).c_str());
    printf("Centralized Focal Epsilon: %.2f\n", problem_input.centralized_focal_epsilon);
    printf("Centralized ASTAR Weight: %.2f\n", problem_input.centralized_astar_weight);
    printf("Centralized Search Time Limit: %.2f\n", problem_input.centralized_search_time_limit);
    printf("Run Decentralized Search: %s\n", problem_input.run_decentralized_search ? "True" : "False");
    if(problem_input.run_decentralized_search) {
        printf("Decentralized Focal Epsilon: %.2f\n", problem_input.decentralized_focal_epsilon);
        printf("Decentralized ASTAR Weight: %.2f\n", problem_input.decentralized_astar_weight);
        printf("Decentralized Search Time Limit: %.2f\n", problem_input.decentralized_search_time_limit);
        printf("Max Decentralized Searches: %d\n", problem_input.max_decentralized_searches);
    }

    return {scenario_config, problem_input};
}

void run(const ScenarioConfig& scenario_config, const ProblemInput& problem_input) {
    auto now = std::chrono::system_clock::now();
    std::time_t curr_time = std::chrono::system_clock::to_time_t(now);
    printf("Starting computation at %s\n", std::ctime(&curr_time));

    Environment env(scenario_config);

    auto start_time = std::chrono::high_resolution_clock::now();

    Lookup lookup;
    precompute_lookup(lookup, scenario_config.map, problem_input.heuristic_type, env.get_agent_positions(), problem_input.run_decentralized_search, problem_input.cell_pruning_method);
    printf("Initial map state: %s\n", get_map_state(lookup, scenario_config.map, env.get_seen(), env.get_agent_positions()).c_str());
    // exit(0);

    int num_strictly_easier = 0;
    for(bool b : lookup.strictly_easier){
        num_strictly_easier += b;
    }
    printf("Number of strictly easier points: %d\n", num_strictly_easier);

    int num_unseen_cells = 0;
    std::string map_state = get_map_state(lookup, scenario_config.map, env.get_seen(), env.get_agent_positions());
    for(int i = 0; i < map_state.size(); i++){
        if(map_state[i] == '0'){
            num_unseen_cells += 1;
        }
    }
    printf("Number of unseen cells at start: %d\n", num_unseen_cells);

    MetricsList aggregated;

    // Calculate makespan optimal path.
    int timestep = 0;
    std::vector<std::vector<Position>> solution = run_heirarchical_search(timestep, env.get_agent_positions(), env.get_seen(), scenario_config.map, problem_input, lookup, aggregated);

    std::ofstream final_run_file;
    final_run_file.open("final_solution.csv");

    // Write Header
    final_run_file << "Timestep, Num Agents, Agent positions, Seen Bitset\n";
    final_run_file << scenario_config.map.map_name << "\n";

    int s_t = 1;
    while(s_t < solution[0].size()){
        std::vector<std::vector<Position>> paths_to_go;
        for(int i = 0; i < solution.size(); i++) {
            std::vector<Position> path_to_go;
            path_to_go.push_back(env.get_agent_positions()[i]);
            for(int t = s_t; t < solution[i].size(); t++) {
                path_to_go.push_back(solution[i][t]);
            }
            paths_to_go.push_back(path_to_go);
        }
        write_run_state_to_file(final_run_file, timestep, paths_to_go, scenario_config.map, env.get_agent_positions(), env.get_seen());

        std::vector<Position> actions;
        for(int i = 0; i < solution.size(); i++) {
            actions.push_back(solution[i][s_t]);
        }
        timestep += 1;
        env.run_action(timestep, actions);
        s_t += 1;
    }

    printf("\n\n\nFinal timestep: %d\n", timestep);
    write_run_state_to_file(final_run_file, timestep, {}, scenario_config.map, env.get_agent_positions(), env.get_seen());

    final_run_file.close();

    Metrics aggregated_sum = aggregated.sum_metrics();
    printf("Aggregated Metrics:\n");
    printf("\tCentralized Search Time: %.3f seconds\n", aggregated_sum.centralized_search_time);
    printf("\tDecentralized Search Time: %.3f seconds\n", aggregated_sum.decentralized_search_time);
    printf("\tNumber of Decentralized Searches: %d\n", aggregated_sum.num_decentralized_searches);
    printf("\tMTSP Calls: %d\n", aggregated_sum.mtsp_total_calls);
    printf("\tMTSP Max Cities: %d\n", aggregated_sum.mtsp_max_cities);
    printf("\tMTSP Setup Time: %.3f seconds\n", aggregated_sum.mtsp_setup_time);
    printf("\tMTSP Solver Time: %.3f seconds\n", aggregated_sum.mtsp_solver_runtime);
    printf("\tLazy f_value Time: %.3f seconds\n", aggregated_sum.lazy_f_value_calculation_time);
    printf("\tTSP f_value Time: %.3f seconds\n", aggregated_sum.tsp_f_value_calculation_time);
    printf("\tExtended Neighbors Calls: %d\n", aggregated_sum.extended_neighbors_calls);
    printf("\tNeighbor Expansion Time: %.3f seconds\n", aggregated_sum.neighbor_expansion_time);
    printf("\tNeighbor Expansion BFS Time: %.3f seconds\n", aggregated_sum.neighbor_expansion_bfs_time);
    printf("\tNeighbor Expansion Pruning Time: %.3f seconds\n", aggregated_sum.neighbor_expansion_pruning_time);
    printf("\tDomination Check Time: %.3f seconds\n", aggregated_sum.domination_check_time);

    printf("Metrics per search call:\n");
    printf("\tCentralized Search Time: %s seconds\n", double_array_to_string(aggregated.centralized_search_time).c_str());
    printf("\tDecentralized Search Time: %s seconds\n", double_array_to_string(aggregated.decentralized_search_time).c_str());
    printf("\tNumber of Decentralized Searches: %s\n", int_array_to_string(aggregated.num_decentralized_searches).c_str());
    printf("\tMTSP Calls: %s\n", int_array_to_string(aggregated.mtsp_total_calls).c_str());
    printf("\tMTSP Setup Time: %s seconds\n", double_array_to_string(aggregated.mtsp_setup_time).c_str());
    printf("\tMTSP Solver Time: %s seconds\n", double_array_to_string(aggregated.mtsp_solver_runtime).c_str());
    printf("\tLazy f_value Time: %s seconds\n", double_array_to_string(aggregated.lazy_f_value_calculation_time).c_str());
    printf("\tTSP f_value Time: %s seconds\n", double_array_to_string(aggregated.tsp_f_value_calculation_time).c_str());
    printf("\tExtended Neighbors Calls: %s\n", int_array_to_string(aggregated.extended_neighbors_calls).c_str());
    printf("\tNeighbor Expansion Time: %s seconds\n", double_array_to_string(aggregated.neighbor_expansion_time).c_str());
    printf("\tNeighbor Expansion BFS Time: %s seconds\n", double_array_to_string(aggregated.neighbor_expansion_bfs_time).c_str());
    printf("\tNeighbor Expansion Pruning Time: %s seconds\n", double_array_to_string(aggregated.neighbor_expansion_pruning_time).c_str());
    printf("\tDomination Check Time: %s seconds\n", double_array_to_string(aggregated.domination_check_time).c_str());

    auto end_time = std::chrono::high_resolution_clock::now();
    std::chrono::duration<double> duration = end_time - start_time;
    printf("Total time taken: %.3f seconds\n", duration.count());
    printf("Total squares seen: %d / %d\n", (int)env.get_seen().count(), scenario_config.map.num_squares);
}

int main(int argc, char** argv) {
    if(argc != 3) {
        std::cerr << "Usage: " << argv[0] << " <scenario_file.json> <solver_config.json>" << std::endl;
        return 1;
    }

    auto [scenario_config, problem_input] = parse_arguments(argc, argv);
    run(scenario_config, problem_input);
    return 0;
}
