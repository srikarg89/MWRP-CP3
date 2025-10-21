#pragma once

#include <vector>
#include <boost/dynamic_bitset.hpp>
#include "shared.hpp"
#include "los.hpp"

class Environment {
public:
    Map map;
    std::vector<Position> agent_positions;
    boost::dynamic_bitset<> seen;
    std::vector<Task> incomplete_tasks;
    std::vector<Task> completed_tasks;
    std::vector<std::vector<Position>> los;

    Environment(const ScenarioConfig& scenario_config) : map(scenario_config.map), agent_positions(scenario_config.agent_starts), incomplete_tasks(scenario_config.tasks) {
        this->seen = boost::dynamic_bitset<>(map.num_squares, 0);

        for(int map_idx = 0; map_idx < map.num_squares; map_idx++){
            Position pos = map.get_pos_from_map_idx(map_idx);
            if(map.check_obstacle(pos)){
                this->los.push_back({});
                this->seen[map_idx] = 1;
            } else {
                switch(map.los_type){
                    case FOUR_WAY_LOS:
                        this->los.push_back(four_way_LOS(pos, map));
                        break;
                    case EIGHT_WAY_LOS:
                        this->los.push_back(eight_way_LOS(pos, map));
                        break;
                    case BRES_LOS:
                        this->los.push_back(bresLOS(pos, map));
                        break;
                }
            }
        }

        for(Position start : agent_positions){
            add_los_to_seen(this->seen, this->los[map.get_map_idx(start)], map);
        }
    }

    void run_action(int timestamp, std::vector<Position> new_positions) {
        for(int i = 0; i < agent_positions.size(); i++){
            agent_positions[i] = new_positions[i];
            auto it = incomplete_tasks.begin();
            while(it != incomplete_tasks.end()){
                if(it->pos.equals(new_positions[i]) && timestamp <= it->deadline){
                    // Task completed.
                    completed_tasks.push_back(*it);
                    it = incomplete_tasks.erase(it);
                    printf("Agent at %s completed a task!\n", new_positions[i].toString().c_str());
                } else {
                    ++it;
                }
            }
            add_los_to_seen(this->seen, this->los[map.get_map_idx(new_positions[i])], map);
        }
    }

    std::vector<Task> get_known_incomplete_tasks() {
        std::vector<Task> known_incomplete_tasks;
        for(Task task : incomplete_tasks){
            if(seen[task.map_idx]){
                known_incomplete_tasks.push_back(task);
            }
        }
        return known_incomplete_tasks;
    }

    std::vector<Task> get_unknown_tasks() {
        std::vector<Task> unknown_tasks;
        for(Task task : incomplete_tasks){
            if(!seen[task.map_idx]){
                unknown_tasks.push_back(task);
            }
        }
        return unknown_tasks;
    }

    std::vector<Task> get_completed_tasks() {
        return completed_tasks;
    }

    std::vector<Position> get_agent_positions() {
        return agent_positions;
    }

    boost::dynamic_bitset<> get_seen() {
        return seen;
    }
};
