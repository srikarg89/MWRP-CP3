#include <iostream>
#include <chrono>
#include <algorithm>
#include "search.hpp"
#include "environment.hpp"


std::tuple<ScenarioConfig, SolverConfig> parse_arguments(int argc, char **argv) {
    // Setup scenario config.
    bool expanding_borders = true; // Use expanding borders optimization by default.
    ScenarioConfig scenario_config = ScenarioConfig::from_json(argv[1]);

    // Setup solver config.
    std::string heuristic_str = argv[2];
    HeuristicType heuristic_type;
    if(heuristic_str == "BFS") {
        heuristic_type = HeuristicType::BFS;
    } else if(heuristic_str == "SINGLETON") {
        heuristic_type = HeuristicType::SINGLETON;
    } else if(heuristic_str == "MST") {
        heuristic_type = HeuristicType::MST;
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
        .expanding_borders = expanding_borders
    };

    return {scenario_config, solver_config};
}

void run(const ScenarioConfig& scenario_config, const SolverConfig& solver_config) {
    Environment env(scenario_config);

    auto start_time = std::chrono::high_resolution_clock::now();

    std::vector<Position> known_tasks = env.get_known_incomplete_tasks();
    printf("Num tasks: %lu\n", known_tasks.size());
    std::vector<std::vector<Position>> solution = run_search(env.get_agent_positions(), known_tasks, env.get_seen(), scenario_config.map, solver_config);

    std::ofstream final_run_file;
    final_run_file.open("final_solution.csv");

    // Write Header
    final_run_file << "Timestep, Num Agents, Agent positions, Seen Bitset\n";
    final_run_file << scenario_config.map_name << "\n";


    int timestep = 0;
    int s_t = 1;
    while(s_t < solution[0].size()){
        std::vector<std::vector<Position>> paths_to_go;
        for(int i = 0; i < solution.size(); i++) {
            std::vector<Position> path_to_go;
            for(int t = s_t; t < solution[i].size(); t++) {
                path_to_go.push_back(solution[i][t]);
            }
            paths_to_go.push_back(path_to_go);
        }
        write_run_state_to_file(final_run_file, timestep, paths_to_go, scenario_config.map, env.get_agent_positions(), env.get_seen(), env.get_known_incomplete_tasks(), env.get_completed_tasks(), env.get_unknown_tasks());

        printf("Running timestep %d\n", timestep);
        std::vector<Position> actions;
        for(int i = 0; i < solution.size(); i++) {
            actions.push_back(solution[i][s_t]);
        }
        env.run_action(actions);
        timestep += 1;
        s_t += 1;

        bool new_task_found = false;
        std::vector<Position> old_known_tasks = known_tasks;
        known_tasks = env.get_known_incomplete_tasks();        
        for(Position pos : known_tasks){
            bool found = false;
            for(Position old_pos : old_known_tasks){
                if(pos.equals(old_pos)){
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
            // TODO: Add in known tasks to the search input.
            printf("New task found! Replanning...\n");
            solution = run_search(env.get_agent_positions(), known_tasks, env.get_seen(), scenario_config.map, solver_config);
            s_t = 1;
        }
    }

    printf("Final timestep: %d\n", timestep);
    write_run_state_to_file(final_run_file, timestep, {}, scenario_config.map, env.get_agent_positions(), env.get_seen(), env.get_known_incomplete_tasks(), env.get_completed_tasks(), env.get_unknown_tasks());

    final_run_file.close();

    auto end_time = std::chrono::high_resolution_clock::now();
    std::chrono::duration<double> duration = end_time - start_time;
    printf("Total time taken: %.3f seconds\n", duration.count());
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
