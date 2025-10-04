#pragma once

#include <vector>
#include <string>

#include <nlohmann/json.hpp>
#include <fstream>

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

    std::string toString() const {
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

enum HeuristicType {
    BFS,
    SINGLETON,
    MST,
    TSP,
    MAX,
    LAZY
};

enum CostType {
    SUM_OF_COSTS,
    MAKESPAN
};

enum MovementType {
    FOUR_WAY_MOVEMENT,
    EIGHT_WAY_MOVEMENT
};

enum LOSType {
    FOUR_WAY_LOS,
    EIGHT_WAY_LOS,
    BRES_LOS        
};


struct Map {
    int x_size;
    int y_size;
    int num_squares;
    std::vector<bool> occupancy; // 1 = occupied, 0 = not occupied.
    std::vector<std::vector<int>> neighbors;

    Map(int x_size, int y_size, const std::vector<bool>& occupancy, MovementType movement) : x_size(x_size), y_size(y_size), occupancy(occupancy) {
        num_squares = x_size * y_size;
        precompute_neighbors(movement);
    }

    int get_map_idx(Position pos) const {
        return pos.y * x_size + pos.x;
    }

    std::vector<int> get_map_idxs(const std::vector<Position>& pos) const {
        std::vector<int> idxs;
        for(const auto& p : pos) {
            idxs.push_back(get_map_idx(p));
        }
        return idxs;
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

    void precompute_neighbors(MovementType movement) {
        // Precompute neighbors.
        neighbors = std::vector<std::vector<int>>(num_squares, std::vector<int>());
        for(int map_idx = 0; map_idx < num_squares; map_idx++){
            Position pos = get_pos_from_map_idx(map_idx);
            if(check_obstacle(pos)){
                continue;
            }
            if(!check_obstacle(pos)) {
                std::vector<Position> pos_neighbors = get_neighbors(pos, movement);
                for(Position nbr : pos_neighbors){
                    neighbors[map_idx].push_back(get_map_idx(nbr));
                }
            }
        }
    }


};

struct Lookup {
    // LOS lookup table. Indexed by map index, gives list of positions visible from that map index.
    std::vector<std::vector<Position>> los;

    // Inverse LOS lookup table. Indexed by map index, gives list of positions that can see that map index.
    std::vector<std::vector<Position>> watchers;

    // All Pairs Shortest Path distances and paths. Indexed by map index.
    std::vector<std::vector<int>> apsp;
    std::vector<std::vector<int>> apsp_paths;

    // Singleton heuristic helper lookup table. Indexed by [source_map_idx][goal_map_idx], gives minimum distance from source to any position that can see goal.
    std::vector<std::vector<int>> min_dist_to_see;

    // Pivot distance lookup tables for MST and TSP heuristics.
    std::vector<std::vector<int>> pivot_cell_dists;
    std::vector<std::vector<int>> pivot_pivot_dists;

    // Sorted order of pivots to consider.
    std::vector<int> sorted_pivot_order;
};

struct DisjointGraph {
    std::vector<int> pivots;
    std::vector<std::vector<int>> pivot_pivot_costs;
    std::vector<std::vector<int>> agent_pivot_costs;
    int max_edge_cost;
};


struct SolverConfig {
    CostType cost_type;
    HeuristicType heuristic_type;
    bool expanding_borders;    
};

inline void strip(std::string& s) {
    s.erase(s.begin(), std::find_if(s.begin(), s.end(), [](unsigned char ch) {
        return !std::isspace(ch);
    }));

    s.erase(std::find_if(s.rbegin(), s.rend(), [](unsigned char ch) {
        return !std::isspace(ch);
    }).base(), s.end());
}

inline std::vector<std::string> splitBySpace(const std::string& str) {
    std::vector<std::string> words;
    std::istringstream iss(str); // Create an input string stream from the string
    std::string word;

    // Read words from the stream, delimited by space
    while (std::getline(iss, word, ' ')) {
        if (!word.empty()) { // Avoid adding empty strings if there are multiple spaces
            words.push_back(word);
        }
    }
    return words;
}

struct ScenarioConfig {
    std::vector<Position> agent_starts;
    Map map;
    MovementType movement_type;
    LOSType los_type;

    static ScenarioConfig from_json(const std::string& config_filename) {
        std::ifstream i(config_filename);
        nlohmann::json parsed_data = nlohmann::json::parse(i);
        auto agents_json = parsed_data["agents"].get<std::vector<std::vector<int>>>();
        std::vector<Position> agent_starts;
        for(const auto& agent : agents_json) {
            if(agent.size() != 2) {
                throw std::runtime_error("Each agent position must have exactly two coordinates.");
            }
            agent_starts.push_back(Position{agent[0], agent[1]});
        }

        // Load in map file.
        std::string map_filename = "../maps/" + parsed_data["map"].get<std::string>();
        std::ifstream inputFile(map_filename);
        if (!inputFile.is_open()) {
            printf("Could not open map file: %s\n", map_filename.c_str());
            exit(0);
        }

        // Create occupancy map.
        std::vector<bool> occupancy = std::vector<bool>();

        // Fill in occupancy map.
        std::string line;
        std::getline(inputFile, line); // Ignore first line.
        std::getline(inputFile, line); strip(line); int y_size = std::stoi(splitBySpace(line)[1]);
        std::getline(inputFile, line); strip(line); int x_size = std::stoi(splitBySpace(line)[1]);
        std::getline(inputFile, line); // Ignore fourth line.

        for (int i = 0; i < y_size; i++) {
            std::getline(inputFile, line); strip(line);
            for(char c : line) {
                occupancy.push_back(c == '@' || c == 'T');
            }
        }

        inputFile.close();
        
        std::string movement_str = parsed_data["movement"].get<std::string>();
        MovementType movement_type;
        if(movement_str == "FOUR_WAY_MOVEMENT") {
            movement_type = FOUR_WAY_MOVEMENT;
        } else if(movement_str == "EIGHT_WAY_MOVEMENT") {
            movement_type = EIGHT_WAY_MOVEMENT;
        } else {
            throw std::runtime_error("Invalid movement type: " + movement_str);
        }

        Map map(x_size, y_size, occupancy, movement_type);

        std::string los_str = parsed_data["los"].get<std::string>();
        LOSType los_type;
        if(los_str == "FOUR_WAY_LOS") {
            los_type = FOUR_WAY_LOS;
        } else if(los_str == "EIGHT_WAY_LOS") {
            los_type = EIGHT_WAY_LOS;
        } else if(los_str == "BRES_LOS") {
            los_type = BRES_LOS;
        } else {
            throw std::runtime_error("Invalid LOS type: " + los_str);
        }

        return {std::move(agent_starts), std::move(map), movement_type, los_type};
    }
};
