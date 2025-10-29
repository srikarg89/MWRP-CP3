#include <iostream>
#include <chrono>
#include <algorithm>
#include <ctime>
#include "search.hpp"
#include "environment.hpp"


std::tuple<ScenarioConfig, SolverConfig> parse_arguments(int argc, char **argv) {
    // Setup scenario config.
    ScenarioConfig scenario_config = ScenarioConfig::from_json(argv[1]);

    // Setup solver config.
    std::string heuristic_str = argv[2];
    HeuristicType heuristic_type;
    if(heuristic_str == "BFS") {
        heuristic_type = HeuristicType::BFS;
    } else if(heuristic_str == "SINGLETON") {
        heuristic_type = HeuristicType::SINGLETON;
    } else if(heuristic_str == "MST") {
        printf("MST heuristic is no longer supported.\n");
        exit(1);
    } else if(heuristic_str == "TSP") {
        heuristic_type = HeuristicType::TSP;
    } else if(heuristic_str == "MAX") {
        heuristic_type = HeuristicType::MAX;
    } else if(heuristic_str == "LAZY") {
        heuristic_type = HeuristicType::LAZY;
    } else {
        throw std::runtime_error("Invalid heuristic type: " + heuristic_str);
    }

    double focal_weight;
    try {
        focal_weight = std::stod(argv[3]);
    } catch (const std::invalid_argument& e) {
        throw std::runtime_error("Invalid focal weight: " + std::string(argv[3]));
    }

    SolverConfig solver_config = SolverConfig{
        .heuristic_type = heuristic_type,
        .focal_weight = focal_weight,
    };

    return {scenario_config, solver_config};
}

void run(const ScenarioConfig& scenario_config, const SolverConfig& solver_config) {
    auto now = std::chrono::system_clock::now();
    std::time_t curr_time = std::chrono::system_clock::to_time_t(now);
    printf("Starting computation at %s\n", std::ctime(&curr_time));

    Environment env(scenario_config);

    auto start_time = std::chrono::high_resolution_clock::now();

    Lookup lookup;
    precompute_lookup(lookup, scenario_config.map, solver_config.heuristic_type, env.get_agent_positions());
    print_map_state(lookup, scenario_config.map, env.get_seen(), env.get_agent_positions());

    std::vector<Task> known_tasks = env.get_known_incomplete_tasks();
    printf("Total number of tasks in problem: %lu\n", known_tasks.size());
    int num_strictly_easier = 0;
    for(bool b : lookup.strictly_easier){
        num_strictly_easier += b;
    }
    printf("Number of strictly easier points: %d\n", num_strictly_easier);

    Metrics aggregated;
    aggregated.reset();

    int timestep = 0;
    std::vector<std::vector<Position>> solution = run_search(timestep, env.get_agent_positions(), known_tasks, env.get_seen(), scenario_config.map, solver_config, lookup);
    aggregated.add(METRICS);

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
            solution = run_search(timestep, env.get_agent_positions(), known_tasks, env.get_seen(), scenario_config.map, solver_config, lookup);
            aggregated.add(METRICS);
            s_t = 1;
        }
    }

    printf("Final timestep: %d\n", timestep);
    write_run_state_to_file(final_run_file, timestep, {}, scenario_config.map, env.get_agent_positions(), env.get_seen(), env.get_known_incomplete_tasks(), env.get_completed_tasks(), env.get_unknown_tasks());

    final_run_file.close();

    printf("Aggregated Metrics:\n");
    printf("\tMTSP Calls: %d\n", aggregated.mtsp_total_calls);
    printf("\tMTSP Setup Time: %.3f seconds\n", aggregated.mtsp_setup_time);
    printf("\tMTSP Solver Time: %.3f seconds\n", aggregated.mtsp_solver_runtime);
    printf("\tGet f_value Time: %.3f seconds\n", aggregated.f_value_calculation_time);
    printf("\tNeighbor Expansion Time: %.3f seconds\n", aggregated.neighbor_expansion_time);
    printf("\tDomination Check Time: %.3f seconds\n", aggregated.domination_check_time);

    auto end_time = std::chrono::high_resolution_clock::now();
    std::chrono::duration<double> duration = end_time - start_time;
    printf("Total time taken: %.3f seconds\n", duration.count());
    printf("Total tasks completed: %lu / %lu\n", env.get_completed_tasks().size(), scenario_config.tasks.size());
    printf("Total squares seen: %d / %d\n", (int)env.get_seen().count(), scenario_config.map.num_squares);
}

int main(int argc, char** argv) {
    if(argc != 4) {
        std::cerr << "Usage: " << argv[0] << " <scenario_file.json> <heuristic_type> <focal_weight>\nExpanding Borders optimization and Makespan cost are used by default." << std::endl;
        return 1;
    }

    auto [scenario_config, solver_config] = parse_arguments(argc, argv);
    run(scenario_config, solver_config);
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

mc_forest, one agent (51, 25), no tasks, 1.25 epsilon focal search
Max time: 450
Total nodes expanded: 35721
Total nodes fully expanded: 35721
Total expansions skipped: 1136
Total generations skipped because of inferior cost: 194758
Total generations skipped because of task failure: 0
Total generations skipped because of task deadlock: 0
Total nodes generated: 126943
MTSP Setup time: 48.892 seconds
MTSP Solver time: 903.050 seconds
Total MTSP calls: 126704
Total neighbor expansion time: 21.571 seconds
Total get_f_value time: 293.306 seconds
Total domination check time: 22.316 seconds
Max node depth expanded: 73
Path 0 length: 450
Total search time taken: 345.360 seconds

Final timestep: 449
Aggregated Metrics:
	MTSP Calls: 126704
	MTSP Setup Time: 48.892 seconds
	MTSP Solver Time: 903.050 seconds
	Get f_value Time: 293.306 seconds
	Neighbor Expansion Time: 21.571 seconds
	Domination Check Time: 22.316 seconds
Total time taken: 346.670 seconds
Total tasks completed: 0 / 0
Total squares seen: 2652 / 2652
*/