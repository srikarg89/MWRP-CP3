#pragma once

#include "shared.hpp"

#include <unordered_map>
#include <unordered_set>
#include <queue>
#include <cstdlib>
#include <climits>
#include <cassert>
#include <algorithm>

namespace pathfinding {
    using node_tuple = std::tuple<int, int, int, int>; // f, g, pos, pred

    int get_heuristic(Position from, Position to){
        return std::max(std::abs(from.x - to.x), std::abs(from.y - to.y));
    }

    std::vector<Position> astar(Position start, Position goal, MovementType movement, Map& map){
        std::unordered_map<int, int> distances(map.x_size * map.y_size, INT_MAX);
        std::unordered_map<int, int> pred_map;
        std::priority_queue<node_tuple, std::vector<node_tuple>, std::greater<node_tuple>> queue;

        int start_heuristic = get_heuristic(start, goal);
        int start_map_idx = map.get_map_idx(start);
        int goal_map_idx = map.get_map_idx(goal);
        queue.push(std::make_tuple(start_heuristic, /*cost = */ 0, start_map_idx, /*pred = */ -1));
        distances[start_map_idx] = 0;

        while(!queue.empty()){
            node_tuple curr = queue.top();
            queue.pop();
            
            int curr_cost = std::get<1>(curr);
            int curr_map_idx = std::get<2>(curr);

            if(distances[curr_map_idx] < curr_cost){
                continue;
            }

            int pred_map_idx = std::get<3>(curr);
            pred_map[curr_map_idx] = pred_map_idx;
            distances[curr_map_idx] = curr_cost;

            if(curr_map_idx == goal_map_idx){
                break;
            }

            Position curr_pos = map.get_pos_from_map_idx(curr_map_idx);
            std::vector<Position> neighbors = map.get_neighbors(curr_pos, movement);
            for(Position neighbor : neighbors){
                int neighbor_cost = curr_cost + 1;
                int neighbor_map_idx = map.get_map_idx(neighbor);
                if(distances[neighbor_map_idx] <= neighbor_cost){
                    continue;
                }
                distances[neighbor_map_idx] = neighbor_cost;
                int neighbor_heuristic = get_heuristic(neighbor, goal);
                queue.push(std::make_tuple(neighbor_cost + neighbor_heuristic, neighbor_cost, neighbor_map_idx, curr_map_idx));
            }
        }

        // Assert that a path was found.
        assert(pred_map.find(goal_map_idx) != pred_map.end());
        
        std::vector<Position> path;
        int map_idx = goal_map_idx;
        while(pred_map[map_idx] != -1){
            path.push_back(map.get_pos_from_map_idx(map_idx));
            map_idx = pred_map[map_idx];
        }
        path.push_back(start);
        
        std::reverse(path.begin(), path.end());
        return path;
    }
}