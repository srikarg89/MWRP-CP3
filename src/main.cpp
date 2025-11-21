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
    printf("Centralized Focal Heuristic Weight: %.2f\n", problem_input.centralized_focal_heuristic_weight);
    printf("Centralized Focal Search Time Limit: %.2f\n", problem_input.centralized_focal_search_time_limit);
    printf("Centralized ASTAR Weight: %.2f\n", problem_input.centralized_astar_weight);
    printf("Run Decentralized Search: %s\n", problem_input.run_decentralized_search ? "True" : "False");
    if(problem_input.run_decentralized_search) {
        printf("Decentralized Focal Epsilon: %.2f\n", problem_input.decentralized_focal_epsilon);
        printf("Decentralized Focal Heuristic Weight: %.2f\n", problem_input.decentralized_focal_heuristic_weight);
        printf("Decentralized Focal Search Time Limit: %.2f\n", problem_input.decentralized_focal_search_time_limit);
        printf("Decentralized ASTAR Weight: %.2f\n", problem_input.decentralized_astar_weight);
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
    std::unordered_map<std::string, std::vector<PastSolution>> solution_history;

    std::vector<Task> known_tasks = env.get_known_incomplete_tasks();
    printf("Total number of tasks in problem: %lu\n", known_tasks.size());
    int num_strictly_easier = 0;
    for(bool b : lookup.strictly_easier){
        num_strictly_easier += b;
    }
    printf("Number of strictly easier points: %d\n", num_strictly_easier);

    MetricsList aggregated;

    int timestep = 0;
    std::vector<std::vector<Position>> solution = run_heirarchical_search(timestep, env.get_agent_positions(), known_tasks, env.get_seen(), scenario_config.map, problem_input, lookup, solution_history, aggregated);

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
        write_run_state_to_file(final_run_file, timestep, paths_to_go, scenario_config.map, env.get_agent_positions(), env.get_seen(), env.get_known_incomplete_tasks(), env.get_completed_tasks(), env.get_unknown_tasks());

        std::vector<Position> actions;
        for(int i = 0; i < solution.size(); i++) {
            actions.push_back(solution[i][s_t]);
        }
        timestep += 1;
        env.run_action(timestep, actions);
        s_t += 1;

        bool new_task_found = false;
        std::vector<Task> old_known_tasks = known_tasks;
        known_tasks = env.get_known_incomplete_tasks();        
        for(Task task : known_tasks){
            bool found = false;
            for(Task old_task : old_known_tasks){
                if(task.id == old_task.id){
                    found = true;
                    break;
                }
            }
            if(!found){
                new_task_found = true;
                break;
            }
        }

        if(new_task_found) {
            printf("New tasks found on timestep %d, recalculating...\n", timestep);
            printf("New task found! Replanning...\n");
            solution = run_heirarchical_search(timestep, env.get_agent_positions(), known_tasks, env.get_seen(), scenario_config.map, problem_input, lookup, solution_history, aggregated);
            s_t = 1;
        }
    }

    printf("\n\n\nFinal timestep: %d\n", timestep);
    write_run_state_to_file(final_run_file, timestep, {}, scenario_config.map, env.get_agent_positions(), env.get_seen(), env.get_known_incomplete_tasks(), env.get_completed_tasks(), env.get_unknown_tasks());

    final_run_file.close();

    Metrics aggregated_sum = aggregated.sum_metrics();
    printf("Aggregated Metrics:\n");
    printf("\tCentralized Search Time: %.3f seconds\n", aggregated_sum.centralized_search_time);
    printf("\tDecentralized Search Time: %.3f seconds\n", aggregated_sum.decentralized_search_time);
    printf("\tNumber of Decentralized Searches: %d\n", aggregated_sum.num_decentralized_searches);
    printf("\tMTSP Calls: %d\n", aggregated_sum.mtsp_total_calls);
    printf("\tMTSP Setup Time: %.3f seconds\n", aggregated_sum.mtsp_setup_time);
    printf("\tMTSP Solver Time: %.3f seconds\n", aggregated_sum.mtsp_solver_runtime);
    printf("\tGet f_value Time: %.3f seconds\n", aggregated_sum.f_value_calculation_time);
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
    printf("\tGet f_value Time: %s seconds\n", double_array_to_string(aggregated.f_value_calculation_time).c_str());
    printf("\tExtended Neighbors Calls: %s\n", int_array_to_string(aggregated.extended_neighbors_calls).c_str());
    printf("\tNeighbor Expansion Time: %s seconds\n", double_array_to_string(aggregated.neighbor_expansion_time).c_str());
    printf("\tNeighbor Expansion BFS Time: %s seconds\n", double_array_to_string(aggregated.neighbor_expansion_bfs_time).c_str());
    printf("\tNeighbor Expansion Pruning Time: %s seconds\n", double_array_to_string(aggregated.neighbor_expansion_pruning_time).c_str());
    printf("\tDomination Check Time: %s seconds\n", double_array_to_string(aggregated.domination_check_time).c_str());

    auto end_time = std::chrono::high_resolution_clock::now();
    std::chrono::duration<double> duration = end_time - start_time;
    printf("Total time taken: %.3f seconds\n", duration.count());
    printf("Total tasks completed: %lu / %lu\n", env.get_completed_tasks().size(), scenario_config.tasks.size());
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

/*
mc_forest, two agents, same start (51, 25), no tasks, 1.5 epsilon focal search
Max time: 284
Total nodes expanded: 18656
Total nodes fully expanded: 18656
Total expansions skipped: 0
Total generations skipped because of inferior cost: 145582
Total generations skipped because of task failure: 0
Total generations skipped because of task deadlock: 0
Total nodes generated: 79743
MTSP Setup time: 29.972 seconds
MTSP Solver time: 304.725 seconds
MTSP Solver time 2: 0.000 seconds
Total MTSP calls: 79651
Total neighbor expansion time: 32.087 seconds
Total get_f_value time: 115.640 seconds
Max node depth expanded: 75
Path 0 length: 284
Path 1 length: 284
Total search time taken: 151.301 seconds

Final timestep: 283
Aggregated Metrics:
	MTSP Calls: 79651
	MTSP Setup Time: 29.972 seconds
	MTSP Solver Time: 304.725 seconds
Total time taken: 152.505 seconds
Total tasks completed: 0 / 0
Total squares seen: 2652 / 2652

mc_forest, two agents, different starts (51, 25) and (2, 25), no tasks, 1.5 epsilon focal search
Max time: 240
Total nodes expanded: 1022
Total nodes fully expanded: 1022
Total expansions skipped: 0
Total generations skipped because of inferior cost: 139513
Total generations skipped because of task failure: 0
Total generations skipped because of task deadlock: 0
Total nodes generated: 38673
MTSP Setup time: 13.914 seconds
MTSP Solver time: 260.933 seconds
MTSP Solver time 2: 0.000 seconds
Total MTSP calls: 38618
Total neighbor expansion time: 0.642 seconds
Total get_f_value time: 22.091 seconds
Max node depth expanded: 42
Path 0 length: 240
Path 1 length: 240
Total search time taken: 23.364 seconds

Final timestep: 239
Aggregated Metrics:
	MTSP Calls: 38618
	MTSP Setup Time: 13.914 seconds
	MTSP Solver Time: 260.933 seconds
Total time taken: 24.610 seconds
Total tasks completed: 0 / 0
Total squares seen: 2652 / 2652

mc_forest, one agent (51, 25), no tasks, 1.25 epsilon focal search with 1.5 focal heuristic weight
Max time: 447
Total nodes expanded: 1114
Total nodes fully expanded: 1114
Total expansions skipped: 112
Total generations skipped because of inferior cost: 3599
Total generations skipped because of task deadlock: 0
Total nodes generated: 3300
MTSP Setup time: 1.410 seconds
MTSP Solver time: 21.946 seconds
Total MTSP calls: 3299
Total neighbor expansion time: 0.278 seconds
Total get_f_value time: 8.297 seconds
Total domination check time: 0.008 seconds
Max node depth expanded: 70
Path 0 length: 447
Total search time taken: 8.777 seconds

Final timestep: 446
Aggregated Metrics:
	MTSP Calls: 3299
	MTSP Setup Time: 1.410 seconds
	MTSP Solver Time: 21.946 seconds
	Get f_value Time: 8.297 seconds
	Neighbor Expansion Time: 0.278 seconds
	Domination Check Time: 0.008 seconds
Total time taken: 9.828 seconds
Total tasks completed: 0 / 0
Total squares seen: 2652 / 2652

mc_forest, one agent (51, 25), no tasks, 1.25 epsilon focal search with 1.5 focal heuristic weight and 20s time limit
Total nodes expanded: 2953
Total nodes fully expanded: 2953
Total expansions skipped: 266
Total generations skipped because of inferior cost: 10496
Total generations skipped because of task deadlock: 0
Total nodes generated: 8590
MTSP Setup time: 3.676 seconds
MTSP Solver time: 45.404 seconds
Total MTSP calls: 8565
Total neighbor expansion time: 1.645 seconds
Total get_f_value time: 17.797 seconds
Total domination check time: 0.027 seconds
Max node depth expanded: 74
Path 0 length: 437
Total search time taken: 20.003 seconds

Final timestep: 436
Aggregated Metrics:
	MTSP Calls: 8565
	MTSP Setup Time: 3.676 seconds
	MTSP Solver Time: 45.404 seconds
	Get f_value Time: 17.797 seconds
	Neighbor Expansion Time: 1.645 seconds
	Domination Check Time: 0.027 seconds
Total time taken: 21.075 seconds
Total tasks completed: 0 / 0
Total squares seen: 2652 / 2652



HEIRARCHICAL SEARCH COMPARISON

mc_forest, 5 tasks, no mrt, two agents, centralized search only.
Final timestep: 361
Aggregated Metrics:
	Centralized Search Time: 148.724 seconds
	Decentralized Search Time: 0.000 seconds
	Number of Decentralized Searches: 0
	MTSP Calls: 139438
	MTSP Setup Time: 47.395 seconds
	MTSP Solver Time: 402.236 seconds
	Get f_value Time: 60.015 seconds
	Neighbor Expansion Time: 66.963 seconds
	Domination Check Time: 1.586 seconds
Total time taken: 170.659 seconds
Total tasks completed: 5 / 5
Total squares seen: 2652 / 2652

mc_forest, 5 tasks, no mrt, two agents, heirarchical search only.
Max time: 102
Task completed at (45,39)!
Task completed at (20,31)!
Final timestep: 275
Aggregated Metrics:
	Centralized Search Time: 83.183 seconds
	Decentralized Search Time: 154.073 seconds
	Number of Decentralized Searches: 12
	MTSP Calls: 110624
	MTSP Setup Time: 36.851 seconds
	MTSP Solver Time: 365.345 seconds
	Get f_value Time: 99.799 seconds
	Neighbor Expansion Time: 102.982 seconds
	Domination Check Time: 2.157 seconds
Total time taken: 274.045 seconds
Total tasks completed: 5 / 5
Total squares seen: 2652 / 2652


*/