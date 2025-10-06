#pragma once

#include <vector>
#include "shared.hpp"

////////////////////////////////
///////// Forms of LOS /////////
////////////////////////////////

// Four-way line of sight (rook movements).
inline std::vector<Position> four_way_LOS(Position pos, const Map& map){
    int dx[] = {-1, 1, 0, 0};
    int dy[] = {0, 0, -1, 1};

    std::vector<Position> los;
    los.push_back(pos);

    for(int i = 0; i < 4; i++){
        Position curr = pos;

        while(true){
            curr = curr.add(dx[i], dy[i]);
            if(map.check_obstacle(curr)){
                break;
            }
            los.push_back(curr);
        }
    }

    return los;
}

// Eight-way line of sight (queen movements).
inline std::vector<Position> eight_way_LOS(Position pos, const Map& map){
    int dx[] = {-1, 1, 0, 0, -1, -1, 1, 1};
    int dy[] = {0, 0, -1, 1, -1, 1, -1, 1};

    std::vector<Position> los;
    los.push_back(pos);

    for(int i = 0; i < 8; i++){
        Position curr = pos;

        while(true){
            curr = curr.add(dx[i], dy[i]);
            if(map.check_obstacle(curr)){
                break;
            }
            los.push_back(curr);
        }
    }

    return los;
}

// Bresham line of sight. Very naive approach. O(N^3) ://
inline bool pairwiseBresLos(Position pos1, Position pos2, const Map& map){
    int dx = std::abs(pos2.x - pos1.x);
    int dy = std::abs(pos2.y - pos1.y);
    int sx = (pos1.x < pos2.x) ? 1 : -1;
    int sy = (pos1.y < pos2.y) ? 1 : -1;
    int err = dx - dy;

    Position curr = pos1;
    while(true){
        if(map.check_obstacle(curr)){
            return false;
        }
        if(curr.equals(pos2)){
            return true;
        }

        int e2 = 2 * err;
        if(e2 > -dy){
            err -= dy;
            curr = curr.add(sx, 0);
        }
        if(e2 < dx){
            err += dx;
            curr = curr.add(0, sy);
        }
    }
    return true;
}


inline std::vector<Position> bresLOS(Position pos, const Map& map){
    std::vector<Position> los;
    for(int x = 0; x < map.x_size; x++){
        for(int y = 0; y < map.y_size; y++){
            Position candidate = {x, y};
            if(pairwiseBresLos(pos, candidate, map)){
                los.push_back(candidate);
            }
        }
    }

    return los;
}

// Returns the number of new squares marked as seen.
inline int add_los_to_seen(boost::dynamic_bitset<>& seen, const std::vector<Position>& los, const Map& map){
    int count = 0;
    for(Position pos : los){
        int map_idx = map.get_map_idx(pos);
        if(!seen[map_idx]){
            count += 1;
        }
        seen[map_idx] = 1;
    }
    return count;
}