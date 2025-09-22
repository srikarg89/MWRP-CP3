#include <iostream>
#include "watchman.hpp"

int main(int argc, char** argv) {
    // Setup scenario config.
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
    } else {
        throw std::runtime_error("Invalid heuristic type: " + heuristic_str);
    }

    std::vector<Position> solution = watchman::run_watchman(scenario_config.agent_starts[0], scenario_config.los_type, scenario_config.map, scenario_config.movement_type, heuristic_type);
    printf("Solution size: %ld\n", solution.size());
    for(Position pos : solution){
        printf("\t%s\n", pos.toString().c_str());
    }
    return 0;
}
