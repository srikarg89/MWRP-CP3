#pragma once

#include "shared.hpp"

// Copy taken since we alter the paths.
inline void add_waits_to_end(std::vector<std::vector<Position>>& paths) {
    int max_time = 0;
    for(const auto& path : paths){
        max_time = std::max(max_time, (int)path.size());
    }

    printf("Max time: %d\n", max_time);
    
    for(int i = 0; i < paths.size(); i++){
        while(paths[i].size() < max_time){
            paths[i].push_back(paths[i].back());
        }
    }
}

inline std::vector<std::vector<Position>> postprocess_collisions(std::vector<std::vector<Position>> paths) {
    int max_time = 0;
    std::vector<int> makespans;
    for(const auto& path : paths){
        makespans.push_back((int)path.size());
        max_time = std::max(max_time, (int)path.size());
    }
    
    for(int i = 0; i < paths.size(); i++){
        while(paths[i].size() < max_time){
            paths[i].push_back(paths[i].back());
        }
    }

    // Resolve point collisions.

    std::vector<std::vector<Position>> new_paths(paths.size(), std::vector<Position>());
    for(int t = 0; t < max_time - 1; t++){
        for(int i = 0; i < paths.size(); i++){
            Position next_pos = paths[i][t + 1];
            std::vector<int> colliding_agents;
            colliding_agents.push_back(i);
            for(int j = i + 1; j < paths.size(); j++){
                if(next_pos.equals(paths[j][t + 1])) {
                    colliding_agents.push_back(j);
                }
            }

            if(colliding_agents.size() <= 1){
                continue;
            }

            printf("Resolving vertex collision between agents %d and %d at time %d at position %s\n", i, colliding_agents[1], t, paths[i][t].toString().c_str());

            int min_agent_makespan = i;
            for(int agent : colliding_agents){
                if(makespans[agent] > makespans[min_agent_makespan]){
                    min_agent_makespan = agent;
                }
            }

            // Have all other agents wait.
            for(int agent : colliding_agents){
                if(agent != min_agent_makespan){
                    Position wait_pos = paths[agent][t];
                    paths[agent].insert(paths[agent].begin() + t + 1, wait_pos);
                }
            }

            // Ensure all paths are of length max_time.
            max_time += 1;
            for(int k = 0; k < paths.size(); k++){
                if(paths[k].size() < max_time){
                    paths[k].push_back(paths[k].back());
                }
            }
        }
    }

    // Resolve edge collisions.
    for(int t = 0; t < max_time - 1; t++){
        for(int i = 0; i < paths.size(); i++){
            for(int j = i + 1; j < paths.size(); j++){
                if(paths[i][t].equals(paths[j][t + 1]) && paths[i][t + 1].equals(paths[j][t])) {
                    printf("Resolving edge collision between agents %d and %d at time %d at position %s\n", i, j, t, paths[i][t].toString().c_str());

                    // Don't go through with the swap.
                    // Swap their future paths from that point on.
                    paths[i].erase(paths[i].begin() + t + 1, paths[i].end());
                    paths[j].erase(paths[j].begin() + t + 1, paths[j].end());
                    for(int k = t + 1; k < max_time; k++){
                        Position temp = paths[i][k];
                        paths[i][k] = paths[j][k];
                        paths[j][k] = temp;
                    }

                    // Ensure that all paths are still of length max_time.
                    paths[i].push_back(paths[i].back());
                    paths[j].push_back(paths[j].back());

                }

            }
        }
    }

    return paths;
}