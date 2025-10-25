#include <iostream>
#include <chrono>
#include <algorithm>
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

    std::string collision_resolution_str = argv[3];
    CollisionResolution collision_resolution;
    if(collision_resolution_str == "NONE") {
        collision_resolution = CollisionResolution::NONE;
    } else if(collision_resolution_str == "POSTPROCESS") {
        collision_resolution = CollisionResolution::POSTPROCESS;
    } else if(collision_resolution_str == "NODE_EXPANSION") {
        collision_resolution = CollisionResolution::NODE_EXPANSION;
    } else {
        throw std::runtime_error("Invalid collision resolution: " + collision_resolution_str);
    }

    SolverConfig solver_config = SolverConfig{
        .heuristic_type = heuristic_type,
        .collision_resolution = collision_resolution,
    };

    return {scenario_config, solver_config};
}

void run(const ScenarioConfig& scenario_config, const SolverConfig& solver_config) {
    Environment env(scenario_config);

    auto start_time = std::chrono::high_resolution_clock::now();

    Lookup lookup;
    precompute_lookup(lookup, scenario_config.map, solver_config.heuristic_type, env.get_agent_positions());

    print_map_state(lookup, scenario_config.map, env.get_seen(), env.get_agent_positions());
    // exit(0);

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

    auto end_time = std::chrono::high_resolution_clock::now();
    std::chrono::duration<double> duration = end_time - start_time;
    printf("Total time taken: %.3f seconds\n", duration.count());
    printf("Total tasks completed: %lu / %lu\n", env.get_completed_tasks().size(), scenario_config.tasks.size());
    printf("Total squares seen: %d / %d\n", (int)env.get_seen().count(), scenario_config.map.num_squares);
}

int main(int argc, char** argv) {
    if(argc != 4) {
        std::cerr << "Usage: " << argv[0] << " <scenario_file.json> <heuristic_type> <collision_resolution>\nExpanding Borders optimization and Makespan cost are used by default." << std::endl;
        return 1;
    }

    auto [scenario_config, solver_config] = parse_arguments(argc, argv);
    run(scenario_config, solver_config);
    return 0;
}
