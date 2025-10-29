#pragma once

#include <vector>
#include <string>
#include <unordered_set>

#include <nlohmann/json.hpp>
#include <fstream>

inline double WEIGHTED_ASTAR_WEIGHT = 1.0;

// Singleton metrics.
struct Metrics {
    // General search metrics.
    int num_skipped_duplicate_node = 0;
    int num_skipped_task_deadline_passed = 0;
    int num_skipped_task_deadlock = 0;
    double neighbor_expansion_time = 0.0;
    double f_value_calculation_time = 0.0;
    double domination_check_time = 0.0;

    // MTSP Metrics.
    double mtsp_setup_time = 0.0;
    double mtsp_solver_runtime = 0.0;
    int mtsp_total_calls = 0;

    void reset() {
        num_skipped_duplicate_node = 0;
        num_skipped_task_deadline_passed = 0;
        num_skipped_task_deadlock = 0;
        neighbor_expansion_time = 0.0;
        f_value_calculation_time = 0.0;
        domination_check_time = 0.0;

        mtsp_setup_time = 0.0;
        mtsp_solver_runtime = 0.0;
        mtsp_total_calls = 0;
    }

    void add(const Metrics& other) {
        num_skipped_duplicate_node += other.num_skipped_duplicate_node;
        num_skipped_task_deadline_passed += other.num_skipped_task_deadline_passed;
        num_skipped_task_deadlock += other.num_skipped_task_deadlock;
        neighbor_expansion_time += other.neighbor_expansion_time;
        f_value_calculation_time += other.f_value_calculation_time;
        domination_check_time += other.domination_check_time;

        mtsp_setup_time += other.mtsp_setup_time;
        mtsp_solver_runtime += other.mtsp_solver_runtime;
        mtsp_total_calls += other.mtsp_total_calls;
    }
};

inline Metrics METRICS;

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

struct AgentState {
    Position pos;
    bool terminated;
    int waiting_idx; // -1 if not waiting, otherwise the index of the task being waited at.
    int cost;

    AgentState(Position p, bool t, int w, int c) : pos(p), terminated(t), waiting_idx(w), cost(c) {}
};

inline std::vector<Position> agent_states_to_positions(const std::vector<AgentState>& agents){
    std::vector<Position> positions;
    for(const AgentState& agent : agents){
        positions.push_back(agent.pos);
    }
    return positions;
}

inline std::vector<AgentState> get_sorted_agents_by_position(const std::vector<AgentState>& agents){
    std::vector<AgentState> sorted_agents = agents;
    std::sort(sorted_agents.begin(), sorted_agents.end(), [](const AgentState& a, const AgentState& b){
        if(a.pos.x != b.pos.x){
            return a.pos.x < b.pos.x;
        }
        if(a.pos.y != b.pos.y){
            return a.pos.y < b.pos.y;
        }
        return a.cost < b.cost;
    });
    return sorted_agents;
}

inline std::string agent_states_to_string(const std::vector<AgentState>& agents){
    std::vector<AgentState> sorted_agents = get_sorted_agents_by_position(agents);
    std::string str = "[";
    for(const AgentState& agent : sorted_agents){
        str += agent.pos.toString() + (agent.terminated ? " / T" : "") + ", ";
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

struct Task {
    int id;
    Position pos;
    int map_idx;
    int deadline;
    int num_agents_required;

    Task(int id, Position p, int map_idx, int deadline, int num_agents_required) : id(id), pos(p), map_idx(map_idx), deadline(deadline), num_agents_required(num_agents_required) {}

    std::string toString() const {
        return "ID: " + std::to_string(id) + ", Pos: " + pos.toString() + ", Map Index: " + std::to_string(map_idx) + ", Deadline: " + std::to_string(deadline) + ", Num Agents Required: " + std::to_string(num_agents_required);
    }
};

inline std::string task_array_hash_string(const std::vector<Task>& tasks){
    std::string str = "[";
    for(const Task& task : tasks){
        str += std::to_string(task.id) + ",";
    }
    str += "]";
    return str;
}

inline std::vector<Position> task_to_pos_array(const std::vector<Task>& tasks){
    std::vector<Position> poses;
    for(const Task& task : tasks){
        poses.push_back(task.pos);
    }
    return poses;
}

inline Task get_task_by_id(const std::vector<Task>& tasks, int id){
    for(const Task& task : tasks){
        if(task.id == id){
            return task;
        }
    }
    throw std::runtime_error("Task with ID " + std::to_string(id) + " not found.");
}

struct Node {
    int node_id;
    std::vector<AgentState> agents;
    std::vector<Task> tasks_left;
    boost::dynamic_bitset<> seen;
    int cost;
    int heuristic;
    int num_seen;
    int f_value;
    int focal_heuristic;
    bool is_lazy;
    int last_agent_expanded;
    int depth;

    Node(int id, std::vector<AgentState> a, boost::dynamic_bitset<> s, std::vector<Task> t, int c, int f, int foc, int n, int l, int d){
        node_id = id;
        agents = a;
        seen = s;
        tasks_left = t;
        cost = c;
        heuristic = f - c;
        num_seen = n;
        f_value = f;
        focal_heuristic = foc;
        last_agent_expanded = l;
        depth = d;
        is_lazy = true;
    }

    void update_f_value(int new_f_value){
        f_value = new_f_value;
        heuristic = f_value - cost;
        is_lazy = false;
    }

    void update_focal_heuristic(int new_focal_heuristic){
        focal_heuristic = new_focal_heuristic;
    }

    bool operator>(const Node& rhs) const {
        int weighted_f = (int)(cost + heuristic * WEIGHTED_ASTAR_WEIGHT);
        int rhs_weighted_f = (int)(rhs.cost + rhs.heuristic * WEIGHTED_ASTAR_WEIGHT);
        return std::tie(weighted_f, heuristic) > std::tie(rhs_weighted_f, rhs.heuristic);
        // return std::tie(f_value, heuristic) > std::tie(rhs.f_value, rhs.heuristic);
    }
};

struct CompareNodeFocal {
    bool operator()(const Node& a, const Node& b) const {
        return a.focal_heuristic > b.focal_heuristic;  // "greater" means min-heap
    }
};

struct VisitedNodeInfo {
    int id;
    std::vector<AgentState> agents;
    boost::dynamic_bitset<> seen;
};

enum HeuristicType {
    BFS,
    SINGLETON,
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

    int get_num_free_squares() const {
        int count = 0;
        for(bool occ : occupancy){
            if(!occ){
                count += 1;
            }
        }
        return count;
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
    std::vector<std::unordered_set<int>> watchers_set;

    // All Pairs Shortest Path distances and paths. Indexed by map index.
    std::vector<std::vector<int>> apsp;
    std::vector<std::vector<int>> apsp_paths;

    // Singleton heuristic helper lookup table. Indexed by [source_map_idx][goal_map_idx], gives minimum distance from source to any position that can see goal.
    std::vector<std::vector<int>> min_dist_to_see;

    // Pivot distance lookup tables for TSP heuristic.
    std::vector<std::vector<int>> pivot_cell_dists;
    std::vector<std::vector<int>> pivot_pivot_dists;

    // Sorted order of pivots to consider.
    std::vector<int> sorted_pivot_order;

    // Squares that are strictly easier to see than another square. Can be ignored during the exploration aspect of the search.
    std::vector<bool> strictly_easier;
};

struct DisjointGraph {
    std::vector<int> pivots;
    std::vector<int> pivot_task_ids; // -1 if exploration pivot, otherwise the task id.
    std::vector<int> num_required_visits;
    std::vector<std::vector<int>> pivot_pivot_costs;
    std::vector<std::vector<int>> agent_pivot_costs;
    int max_edge_cost;
    int num_exploration_pivots;
};

inline void print_disjoint_graph(const DisjointGraph& graph) {
    printf("Disjoint Graph:\n");
    printf("Pivots: %s\n", int_array_to_string(graph.pivots).c_str());
    printf("Pivot-Pivot Costs:\n");
    for(const auto& row : graph.pivot_pivot_costs){
        printf("%s\n", int_array_to_string(row).c_str());
    }
    printf("Agent-Pivot Costs:\n");
    for(const auto& row : graph.agent_pivot_costs){
        printf("%s\n", int_array_to_string(row).c_str());
    }
    printf("Max Edge Cost: %d\n", graph.max_edge_cost);
    printf("Num Exploration Pivots: %d\n", graph.num_exploration_pivots);
}

struct SolverConfig {
    HeuristicType heuristic_type;
    CollisionResolution collision_resolution;
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
    std::vector<Task> tasks;
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

        auto tasks_json = parsed_data["tasks"];
        std::vector<Task> tasks;
        for(const auto& task : tasks_json) {
            int task_id = task["id"].get<int>();
            auto task_position = task["location"].get<std::vector<int>>();
            if(task_position.size() != 2) {
                throw std::runtime_error("Each task position must have exactly two coordinates.");
            }
            int deadline = INT_MAX;
            int num_agents_required = 1;
            if(task.contains("deadline")) {
                deadline = task["deadline"].get<int>();
            }
            if(task.contains("num_agents_required")) {
                num_agents_required = task["num_agents_required"].get<int>();
            }
            Position task_pos = Position{task_position[0], task_position[1]};
            tasks.push_back(Task(task_id, task_pos, map.get_map_idx(task_pos), deadline, num_agents_required));
        }

        // Validate agent and task positions.
        for(const auto& agent : agent_starts) {
            if(map.get_map_idx(agent) < 0 || map.get_map_idx(agent) >= map.num_squares || map.check_obstacle(agent)) {
                throw std::runtime_error("Agent position " + agent.toString() + " is out of bounds or on an obstacle.");
            }
        }
        for(const auto& task : tasks) {
            if(task.map_idx < 0 || task.map_idx >= map.num_squares || map.check_obstacle(task.pos)) {
                throw std::runtime_error("Task (" + task.toString() + ") is out of bounds or on an obstacle.");
            }
        }

        return {std::move(agent_starts), std::move(tasks), std::move(map)};
    }
};
