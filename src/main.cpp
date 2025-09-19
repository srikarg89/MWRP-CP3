#include <iostream>
#include "watchman.hpp"

Map create_example_map(){
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
    Map map = create_example_map();
    Position start_pos = {1, 2};
    watchman::LOSType los_type = watchman::LOSType::FOUR_WAY_LOS;
    std::vector<Position> solution = watchman::run_watchman(start_pos, los_type, map);
    printf("Solution:\n");
    for(Position pos : solution){
        printf("\t%s\n", pos.toString().c_str());
    }
    return 0;
}
