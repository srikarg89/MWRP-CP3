#include <iostream>
#include <chrono>
#include "watchman.hpp"

int main(int argc, char** argv) {
    // Setup scenario config.
    // bool expanding_borders = false;
    // if(argc == 5 && std::string(argv[4]) == "EB"){
    //     expanding_borders = true;
    // }
    bool expanding_borders = true; // Use expanding borders optimization by default.
    if(argc != 4) {
        std::cerr << "Usage: " << argv[0] << " <scenario_file.json> <heuristic_type> <collision_resolution>\nExpanding Borders optimization and Makespan cost are used by default." << std::endl;
        return 1;
    }
    ScenarioConfig scenario_config = ScenarioConfig::from_json(argv[1]);
    MovementType movement_type = MovementType::FOUR_WAY_MOVEMENT;

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

    auto start_time = std::chrono::high_resolution_clock::now();

    auto solution = watchman::run_watchman(scenario_config.agent_starts, scenario_config, solver_config);
    printf("Solution size: %ld\n", solution.size());
    // for(const std::vector<Position>& agent_positions : solution){
    //     printf("\t");
    //     for(const Position& pos : agent_positions){
    //         printf("%s ", pos.toString().c_str());
    //     }
    //     printf("\n");
    // }

    auto end_time = std::chrono::high_resolution_clock::now();
    std::chrono::duration<double> duration = end_time - start_time;
    printf("Total time taken: %.3f seconds\n", duration.count());

    return 0;
}
