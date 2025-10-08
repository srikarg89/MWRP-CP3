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
    std::vector<Position> incomplete_tasks;
    std::vector<Position> completed_tasks;
    std::vector<std::vector<Position>> los;

    Environment(const ScenarioConfig& scenario_config) : map(scenario_config.map), agent_positions(scenario_config.agent_starts), incomplete_tasks(scenario_config.task_locations) {
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

    void run_action(std::vector<Position> new_positions) {
        for(int i = 0; i < agent_positions.size(); i++){
            agent_positions[i] = new_positions[i];
            auto it = std::find(incomplete_tasks.begin(), incomplete_tasks.end(), new_positions[i]);
            if(it != incomplete_tasks.end()){
                // Task completed.
                completed_tasks.push_back(*it);
                incomplete_tasks.erase(it);
                printf("Agent at %s completed a task!\n", new_positions[i].toString().c_str());
            }
            add_los_to_seen(this->seen, this->los[map.get_map_idx(new_positions[i])], map);
        }
    }

    std::vector<Position> get_known_incomplete_tasks() {
        std::vector<Position> known_incomplete_tasks;
        for(Position task : incomplete_tasks){
            if(seen[map.get_map_idx(task)]){
                known_incomplete_tasks.push_back(task);
            }
        }
        return known_incomplete_tasks;
    }

    std::vector<Position> get_unknown_tasks() {
        std::vector<Position> unknown_tasks;
        for(Position task : incomplete_tasks){
            if(!seen[map.get_map_idx(task)]){
                unknown_tasks.push_back(task);
            }
        }
        return unknown_tasks;
    }

    std::vector<Position> get_completed_tasks() {
        return completed_tasks;
    }

    std::vector<Position> get_agent_positions() {
        return agent_positions;
    }

    boost::dynamic_bitset<> get_seen() {
        return seen;
    }
};
