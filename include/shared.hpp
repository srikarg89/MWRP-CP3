#pragma once

#include <vector>
#include <string>

#include <nlohmann/json.hpp>
#include <fstream>

inline double WEIGHTED_ASTAR_WEIGHT = 1.5;

// General, shared types.

struct Position {
    int x;
    int y;

    bool operator==(const Position& other) const {
        return (x == other.x && y == other.y);
    }

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

inline std::string int_array_to_string(const std::vector<int>& arr){
    std::string str = "[";
    for(int i : arr){
        str += std::to_string(i) + ", ";
    }
    str += "]";
    return str;
}

struct AgentState{
    Position pos;
    bool terminated;
    int cost;

    AgentState(Position p, bool t, int c) : pos(p), terminated(t), cost(c) {}
};

inline std::vector<Position> agent_states_to_positions(const std::vector<AgentState>& agents){
    std::vector<Position> positions;
    for(const AgentState& agent : agents){
        positions.push_back(agent.pos);
    }
    return positions;
}

// Used for hashing visited nodes.
inline std::string agent_states_to_string(const std::vector<AgentState>& agents){
    std::string str = "[";
    for(const AgentState& agent : agents){
        str += agent.pos.toString() + " / " + (agent.terminated ? " / T" : "") + ", ";
    }
    str += "]";
    return str;
}

// Used for printing debug information.
inline std::string agent_states_to_print_string(const std::vector<AgentState>& agents){
    std::string str = "[";
    for(const AgentState& agent : agents){
        str += agent.pos.toString() + " / " + std::to_string(agent.cost) + (agent.terminated ? " / T" : "") + ", ";
    }
    str += "]";
    return str;
}

struct Node {
    int node_id;
    std::vector<AgentState> agents;
    std::vector<int> tasks_left;
    boost::dynamic_bitset<> seen;
    int cost;
    int heuristic;
    int num_seen;
    int f_value;
    bool is_lazy;

    Node(int id, std::vector<AgentState> a, boost::dynamic_bitset<> s, std::vector<int> t, int c, int f, int n){
        node_id = id;
        agents = a;
        seen = s;
        tasks_left = t;
        cost = c;
        heuristic = f - c;
        num_seen = n;
        f_value = f;
        is_lazy = true;
    }

    void update_f_value(int new_f_value){
        f_value = new_f_value;
        heuristic = f_value - cost;
        is_lazy = false;
    }

    bool operator>(const Node& rhs) const {
        int weighted_f = (int)(cost + heuristic * WEIGHTED_ASTAR_WEIGHT);
        int rhs_weighted_f = (int)(rhs.cost + rhs.heuristic * WEIGHTED_ASTAR_WEIGHT);
        return std::tie(weighted_f, heuristic) > std::tie(rhs_weighted_f, rhs.heuristic);
        // return std::tie(f_value, heuristic) > std::tie(rhs.f_value, rhs.heuristic);
    }
};

enum HeuristicType {
    BFS,
    SINGLETON,
    MST,
    TSP,
    MAX,
    LAZY
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

enum CollisionResolution {
    NONE,
    POSTPROCESS,
    NODE_EXPANSION
};


struct Map {
    std::string map_name;
    int x_size;
    int y_size;
    int num_squares;
    std::vector<bool> occupancy; // 1 = occupied, 0 = not occupied.
    std::vector<std::vector<int>> neighbors;
    MovementType movement_type;
    LOSType los_type;

    Map(std::string name, int x_size, int y_size, const std::vector<bool>& occupancy, MovementType movement, LOSType los) : map_name(name), x_size(x_size), y_size(y_size), occupancy(occupancy), movement_type(movement), los_type(los) {
        num_squares = x_size * y_size;
        precompute_neighbors();
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

    std::vector<Position> get_neighbors(Position pos) const {
        int dX[] = {-1, 1, 0, 0, -1, -1, 1, 1};
        int dY[] = {0, 0, -1, 1, -1, 1, -1, 1};
        int cutoff = (movement_type == FOUR_WAY_MOVEMENT) ? 4 : 8;

        std::vector<Position> neighbors;
        for(int i = 0; i < cutoff; i++){
            Position new_pos = pos.add(dX[i], dY[i]);
            if(!check_obstacle(new_pos)){
                neighbors.push_back(new_pos);
            }
        }
        return neighbors;
    }

    void precompute_neighbors() {
        // Precompute neighbors.
        neighbors = std::vector<std::vector<int>>(num_squares, std::vector<int>());
        for(int map_idx = 0; map_idx < num_squares; map_idx++){
            Position pos = get_pos_from_map_idx(map_idx);
            if(check_obstacle(pos)){
                continue;
            }
            if(!check_obstacle(pos)) {
                std::vector<Position> pos_neighbors = get_neighbors(pos);
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
    int num_exploration_pivots;
};


struct SolverConfig {
    HeuristicType heuristic_type;
    CollisionResolution collision_resolution;
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
    std::vector<Position> task_locations;
    Map map;

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

        auto tasks_json = parsed_data["tasks"].get<std::vector<std::vector<int>>>();
        std::vector<Position> task_locations;
        for(const auto& task : tasks_json) {
            if(task.size() != 2) {
                throw std::runtime_error("Each task position must have exactly two coordinates.");
            }
            task_locations.push_back(Position{task[0], task[1]});
        }

        // Load in map file.
        std::string map_name = parsed_data["map"].get<std::string>();
        std::string map_filename = "../maps/" + map_name;
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

        Map map(map_name, x_size, y_size, occupancy, movement_type, los_type);

        // Validate agent and task positions.
        for(const auto& agent : agent_starts) {
            if(map.get_map_idx(agent) < 0 || map.get_map_idx(agent) >= map.num_squares || map.check_obstacle(agent)) {
                throw std::runtime_error("Agent position " + agent.toString() + " is out of bounds or on an obstacle.");
            }
        }
        for(const auto& task : task_locations) {
            if(map.get_map_idx(task) < 0 || map.get_map_idx(task) >= map.num_squares || map.check_obstacle(task)) {
                throw std::runtime_error("Task position " + task.toString() + " is out of bounds or on an obstacle.");
            }
        }

        return {std::move(agent_starts), std::move(task_locations), std::move(map)};
    }
};
