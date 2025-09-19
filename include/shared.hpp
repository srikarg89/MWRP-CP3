#pragma once

#include <vector>
#include <string>

// General, shared types.

struct Position {
    int x;
    int y;

    inline Position add(int dx, int dy) {
        return Position {
            .x=x + dx,
            .y=y + dy
        };
    }

    inline bool equals(const Position& other) {
        return x == other.x && y == other.y;
    }

    std::string toString(){
        return "(" + std::to_string(x) + "," + std::to_string(y) + ")";
    }
};

inline std::string pos_array_to_string(const std::vector<Position>& poses){
    std::string str = "[";
    for(Position pos : poses){
        str += pos.toString() + ", ";
    }
    str += "]";
    return str;
}

enum MovementType {
    FOUR_WAY_MOVEMENT,
    EIGHT_WAY_MOVEMENT
};

// TODO: Weighted map.
struct Map {
    int x_size;
    int y_size;
    std::vector<bool> occupancy; // 1 = occupied, 0 = not occupied.

    int get_map_idx(Position pos) const {
        return pos.y * x_size + pos.x;
    }

    Position get_pos_from_map_idx(int map_idx) const {
        return Position { 
            .x=map_idx % x_size,
            .y=map_idx / x_size
        };
    }

    bool check_obstacle(Position pos) const {
        return pos.x < 0 || pos.x >= x_size || pos.y < 0 || pos.y >= y_size || occupancy[get_map_idx(pos)];
    }

    // TODO: Can probably also be precomputed for bonus speedup.
    std::vector<Position> get_neighbors(Position pos, MovementType movement) const {
        int dX[] = {-1, 1, 0, 0, -1, -1, 1, 1};
        int dY[] = {0, 0, -1, 1, -1, 1, -1, 1};
        int cutoff = (movement == FOUR_WAY_MOVEMENT) ? 4 : 8;

        std::vector<Position> neighbors;
        for(int i = 0; i < cutoff; i++){
            Position new_pos = pos.add(dX[i], dY[i]);
            if(!check_obstacle(new_pos)){
                neighbors.push_back(new_pos);
            }
        }
        return neighbors;
    }
};
