#include <iostream>
#include <chrono>
#include "watchman.hpp"

int main(int argc, char** argv) {
    // Setup scenario config.
    bool expanding_borders = false;
    if(argc == 5 && std::string(argv[4]) == "EB"){
        expanding_borders = true;
    }
    else if(argc != 4) {
        std::cerr << "Usage: " << argv[0] << " <scenario_file.json> <cost_type> <heuristic_type>\nOptionally add in EB as a fourth argument to use Expanding Borders optimization" << std::endl;
        return 1;
    }
    ScenarioConfig scenario_config = ScenarioConfig::from_json(argv[1]);
    MovementType movement_type = MovementType::FOUR_WAY_MOVEMENT;

    // Setup solver config.
    std::string cost_str = argv[2];
    CostType cost_type;
    if(cost_str == "SOC") {
        cost_type = CostType::SUM_OF_COSTS;
    } else if(cost_str == "MKSP") {
        cost_type = CostType::MAKESPAN;
    } else {
        throw std::runtime_error("Invalid cost type: " + cost_str);
    }

    std::string heuristic_str = argv[3];
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

    SolverConfig solver_config = SolverConfig{
        .cost_type = cost_type,
        .heuristic_type = heuristic_type,
        .expanding_borders = expanding_borders
    };

    auto solution = watchman::run_watchman(scenario_config.agent_starts, scenario_config, solver_config);
    printf("Solution size: %ld\n", solution.size());
    // for(const std::vector<Position>& agent_positions : solution){
    //     printf("\t");
    //     for(const Position& pos : agent_positions){
    //         printf("%s ", pos.toString().c_str());
    //     }
    //     printf("\n");
    // }

    return 0;
}
