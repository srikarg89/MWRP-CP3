#include <iostream>
#include "watchman.hpp"

Map create_figure_1_map(){
    int x_size = 5;
    int y_size = 5;
    std::vector<bool> occupancy = {
        0, 0, 0, 0, 0,
        1, 1, 0, 1, 1,
        0, 0, 0, 0, 0,
        0, 1, 1, 1, 0,
        0, 1, 0, 0, 0
    };
    return Map {
        .x_size=x_size,
        .y_size=y_size,
        .occupancy=occupancy
    };
}

Map create_figure_3_map(){
    int x_size = 4;
    int y_size = 5;
    std::vector<bool> occupancy = {
        1, 0, 0, 0,
        1, 0, 1, 1,
        0, 0, 0, 0,
        0, 1, 1, 0,
        0, 0, 0, 0
    };
    return Map {
        .x_size=x_size,
        .y_size=y_size,
        .occupancy=occupancy
    };
}

int main() {
    // watchman::HeuristicType heuristic_type = watchman::HeuristicType::BFS;
    // watchman::HeuristicType heuristic_type = watchman::HeuristicType::SINGLETON;
    watchman::HeuristicType heuristic_type = watchman::HeuristicType::MIN_SPAN;
    MovementType movement_type = MovementType::FOUR_WAY_MOVEMENT;

    // Figure 1 example
    // Map map = create_figure_1_map();
    // Position start_pos = {0, 4};
    // watchman::LOSType los_type = watchman::LOSType::FOUR_WAY_LOS;
    // watchman::LOSType los_type = watchman::LOSType::BRES_LOS;

    // Figure 3 example
    Map map = create_figure_3_map();
    Position start_pos = {1, 2};
    watchman::LOSType los_type = watchman::LOSType::FOUR_WAY_LOS;

    std::vector<Position> solution = watchman::run_watchman(start_pos, los_type, map, movement_type, heuristic_type);
    printf("Solution:\n");
    for(Position pos : solution){
        printf("\t%s\n", pos.toString().c_str());
    }
    return 0;
}
