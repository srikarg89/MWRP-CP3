#pragma once

#include <vector>
#include <string>
#include <unordered_set>
#include <boost/dynamic_bitset.hpp>

#include <nlohmann/json.hpp>
#include <fstream>

inline double A_STAR_WEIGHT = 0.0;

// Singleton metrics.

inline std::string int_array_to_string(const std::vector<int>& arr) {
    std::string result = "[";
    for(int i = 0; i < arr.size(); i++){
        result += std::to_string(arr[i]);
        if(i != arr.size() - 1){
            result += ", ";
        }
    }
    result += "]";
    return result;
}

inline std::string double_array_to_string(const std::vector<double>& arr) {
    std::string result = "[";
    for(int i = 0; i < arr.size(); i++){
        result += std::to_string(arr[i]);
        if(i != arr.size() - 1){
            result += ", ";
        }
    }
    result += "]";
    return result;
}

struct Metrics {
    // Heirarchical search metrics.
    double centralized_search_time = 0.0;
    double decentralized_search_time = 0.0;
    int num_decentralized_searches = 0;

    // General search metrics.
    int num_skipped_duplicate_node = 0;
    int num_skipped_task_deadlock = 0;
    int num_skipped_high_lazy_f_value = 0;
    int extended_neighbors_calls = 0;
    double neighbor_expansion_time = 0.0;
    double lazy_f_value_calculation_time = 0.0;
    double tsp_f_value_calculation_time = 0.0;
    double domination_check_time = 0.0;
    double neighbor_expansion_bfs_time = 0.0;
    double neighbor_expansion_pruning_time = 0.0;
    double adding_los_time = 0.0;

    // MTSP Metrics.
    double mtsp_setup_time = 0.0;
    double mtsp_solver_runtime = 0.0;
    int mtsp_total_calls = 0;

    void reset() {
        centralized_search_time = 0.0;
        decentralized_search_time = 0.0;
        num_decentralized_searches = 0;

        num_skipped_duplicate_node = 0;
        num_skipped_task_deadlock = 0;
        num_skipped_high_lazy_f_value = 0;
        extended_neighbors_calls = 0;
        neighbor_expansion_time = 0.0;
        lazy_f_value_calculation_time = 0.0;
        tsp_f_value_calculation_time = 0.0;
        domination_check_time = 0.0;
        neighbor_expansion_bfs_time = 0.0;
        neighbor_expansion_pruning_time = 0.0;
        adding_los_time = 0.0;

        mtsp_setup_time = 0.0;
        mtsp_solver_runtime = 0.0;
        mtsp_total_calls = 0;
    }

    void add(const Metrics& other) {
        centralized_search_time += other.centralized_search_time;
        decentralized_search_time += other.decentralized_search_time;
        num_decentralized_searches += other.num_decentralized_searches;

        num_skipped_duplicate_node += other.num_skipped_duplicate_node;
        num_skipped_task_deadlock += other.num_skipped_task_deadlock;
        num_skipped_high_lazy_f_value += other.num_skipped_high_lazy_f_value;
        extended_neighbors_calls += other.extended_neighbors_calls;
        neighbor_expansion_time += other.neighbor_expansion_time;
        neighbor_expansion_bfs_time += other.neighbor_expansion_bfs_time;
        neighbor_expansion_pruning_time += other.neighbor_expansion_pruning_time;
        lazy_f_value_calculation_time += other.lazy_f_value_calculation_time;
        tsp_f_value_calculation_time += other.tsp_f_value_calculation_time;
        domination_check_time += other.domination_check_time;
        adding_los_time += other.adding_los_time;

        mtsp_setup_time += other.mtsp_setup_time;
        mtsp_solver_runtime += other.mtsp_solver_runtime;
        mtsp_total_calls += other.mtsp_total_calls;
    }
};

struct MetricsList {
    // Heirarchical search metrics.
    std::vector<double> centralized_search_time;
    std::vector<double> decentralized_search_time;
    std::vector<int> num_decentralized_searches;

    // General search metrics.
    std::vector<int> num_skipped_duplicate_node;
    std::vector<int> num_skipped_task_deadlock;
    std::vector<int> num_skipped_high_lazy_f_value;
    std::vector<int> extended_neighbors_calls;
    std::vector<double> neighbor_expansion_time;
    std::vector<double> lazy_f_value_calculation_time;
    std::vector<double> tsp_f_value_calculation_time;
    std::vector<double> domination_check_time;
    std::vector<double> neighbor_expansion_bfs_time;
    std::vector<double> neighbor_expansion_pruning_time;

    // MTSP Metrics.
    std::vector<double> mtsp_setup_time;
    std::vector<double> mtsp_solver_runtime;
    std::vector<int> mtsp_total_calls;

    void add_metrics(const Metrics& metrics) {
        num_skipped_duplicate_node.push_back(metrics.num_skipped_duplicate_node);
        num_skipped_task_deadlock.push_back(metrics.num_skipped_task_deadlock);
        num_skipped_high_lazy_f_value.push_back(metrics.num_skipped_high_lazy_f_value);
        extended_neighbors_calls.push_back(metrics.extended_neighbors_calls);
        neighbor_expansion_time.push_back(metrics.neighbor_expansion_time);
        neighbor_expansion_bfs_time.push_back(metrics.neighbor_expansion_bfs_time);
        neighbor_expansion_pruning_time.push_back(metrics.neighbor_expansion_pruning_time);
        lazy_f_value_calculation_time.push_back(metrics.lazy_f_value_calculation_time);
        tsp_f_value_calculation_time.push_back(metrics.tsp_f_value_calculation_time);
        domination_check_time.push_back(metrics.domination_check_time);

        mtsp_setup_time.push_back(metrics.mtsp_setup_time);
        mtsp_solver_runtime.push_back(metrics.mtsp_solver_runtime);
        mtsp_total_calls.push_back(metrics.mtsp_total_calls);
    }
    
    Metrics sum_metrics() {
        Metrics metrics;
        metrics.centralized_search_time = std::accumulate(centralized_search_time.begin(), centralized_search_time.end(), 0.0);
        metrics.decentralized_search_time = std::accumulate(decentralized_search_time.begin(), decentralized_search_time.end(), 0.0);
        metrics.num_decentralized_searches = std::accumulate(num_decentralized_searches.begin(), num_decentralized_searches.end(), 0);

        metrics.num_skipped_duplicate_node = std::accumulate(num_skipped_duplicate_node.begin(), num_skipped_duplicate_node.end(), 0);
        metrics.num_skipped_task_deadlock = std::accumulate(num_skipped_task_deadlock.begin(), num_skipped_task_deadlock.end(), 0);
        metrics.num_skipped_high_lazy_f_value = std::accumulate(num_skipped_high_lazy_f_value.begin(), num_skipped_high_lazy_f_value.end(), 0);
        metrics.extended_neighbors_calls = std::accumulate(extended_neighbors_calls.begin(), extended_neighbors_calls.end(), 0);
        metrics.neighbor_expansion_time = std::accumulate(neighbor_expansion_time.begin(), neighbor_expansion_time.end(), 0.0);
        metrics.neighbor_expansion_bfs_time = std::accumulate(neighbor_expansion_bfs_time.begin(), neighbor_expansion_bfs_time.end(), 0.0);
        metrics.neighbor_expansion_pruning_time = std::accumulate(neighbor_expansion_pruning_time.begin(), neighbor_expansion_pruning_time.end(), 0.0);
        metrics.lazy_f_value_calculation_time = std::accumulate(lazy_f_value_calculation_time.begin(), lazy_f_value_calculation_time.end(), 0.0);
        metrics.tsp_f_value_calculation_time = std::accumulate(tsp_f_value_calculation_time.begin(), tsp_f_value_calculation_time.end(), 0.0);
        metrics.domination_check_time = std::accumulate(domination_check_time.begin(), domination_check_time.end(), 0.0);

        metrics.mtsp_setup_time = std::accumulate(mtsp_setup_time.begin(), mtsp_setup_time.end(), 0.0);
        metrics.mtsp_solver_runtime = std::accumulate(mtsp_solver_runtime.begin(), mtsp_solver_runtime.end(), 0.0);
        metrics.mtsp_total_calls = std::accumulate(mtsp_total_calls.begin(), mtsp_total_calls.end(), 0);
        
        return metrics;
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

inline std::string bit_set_to_string(const boost::dynamic_bitset<>& bitset){
    std::string str = "";
    for(size_t i = 0; i < bitset.size(); i++){
        str += bitset[i] ? "1" : "0";
    }
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
        // TODO: Change this if we expand individual agents at a time.
        // Cost matters for distinguishing states when waiting.
        // str += agent.pos.toString() + (agent.terminated ? " / T" : "") + " / " + std::to_string(agent.waiting_idx) + (agent.waiting_idx != -1 ? " / " + std::to_string(agent.cost) : "") + ", ";
        // str += agent.pos.toString() + ", ";
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
    int num_agents_required;
    int release_time;
    int deadline;

    Task(int id, Position p, int map_idx, int num_agents_required) : id(id), pos(p), map_idx(map_idx), num_agents_required(num_agents_required) {
        release_time = 0;
        deadline = std::numeric_limits<int>::max();
    }

    std::string toString() const {
        return "ID: " + std::to_string(id) + ", Pos: " + pos.toString() + ", Map Index: " + std::to_string(map_idx) + ", Num Agents Required: " + std::to_string(num_agents_required);
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

inline bool task_arrays_equal(std::vector<Task> a, std::vector<Task> b){
    if(a.size() != b.size()){
        return false;
    }
    std::sort(a.begin(), a.end(), [](const Task& t1, const Task& t2){ return t1.id < t2.id; });
    std::sort(b.begin(), b.end(), [](const Task& t1, const Task& t2){ return t1.id < t2.id; });
    for(int i = 0; i < a.size(); i++){
        if(a[i].id != b[i].id || a[i].release_time != b[i].release_time || a[i].deadline != b[i].deadline || a[i].num_agents_required != b[i].num_agents_required){
            return false;
        }
    }
    return true;
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
    int depth;

    Node(int id, std::vector<AgentState> a, boost::dynamic_bitset<> s, std::vector<Task> t, int c, int f, int foc, int n, int d){
        node_id = id;
        agents = a;
        seen = s;
        tasks_left = t;
        cost = c;
        heuristic = f - c;
        num_seen = n;
        f_value = f;
        focal_heuristic = foc;
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
        return std::tie(f_value, heuristic, node_id) > std::tie(rhs.f_value, rhs.heuristic, rhs.node_id);
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

inline std::string heuristic_type_to_string(HeuristicType ht){
    switch(ht){
        case BFS:
            return "BFS";
        case SINGLETON:
            return "SINGLETON";
        case TSP:
            return "TSP";
        case MAX:
            return "MAX";
        case LAZY:
            return "LAZY";
        default:
            return "UNKNOWN_HEURISTIC_TYPE";
    }
}

enum FocalMethod {
    SOC,
    MOC
};

inline std::string focal_method_to_string(FocalMethod fm){
    switch(fm){
        case SOC:
            return "SOC";
        case MOC:
            return "MOC";
        default:
            return "UNKNOWN_FOCAL_METHOD";
    }
}

enum CellPruningMethod {
    NONE,
    CELL_DOMINATION,
    PATH_DOMINATION,
    CELL_THEN_PATH_DOMINATION
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
    std::vector<std::vector<bool>> strictly_easier_per_agent;
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

struct PastSolution {
    std::vector<Position> path;
    boost::dynamic_bitset<> seen;
    std::vector<Task> tasks_left;
};

struct Optimizations {
    bool prune_pivots;
    bool run_parallel;
    bool expand_lowest_cost_agent_only;
    int max_pivots_after_pruning;
    int max_pivots_generated;
    int parallel_batch_size;
};

struct ProblemInput {
    HeuristicType heuristic_type;
    FocalMethod focal_method;
    CellPruningMethod cell_pruning_method;
    Optimizations optimizations;
    double centralized_focal_epsilon;
    double centralized_focal_heuristic_weight;
    double centralized_search_time_limit;
    double centralized_hard_search_time_limit;
    double centralized_astar_weight;
    bool run_decentralized_search;
    double decentralized_focal_epsilon;
    double decentralized_focal_heuristic_weight;
    double decentralized_search_time_limit;
    double decentralized_hard_search_time_limit;
    double decentralized_astar_weight;
    int max_decentralized_searches;

    static ProblemInput from_json(const std::string& config_filename) {
        std::ifstream i(config_filename);
        nlohmann::json parsed_data = nlohmann::json::parse(i);
        HeuristicType heuristic_type;
        std::string heuristic_str = parsed_data["heuristic"].get<std::string>();
        if(heuristic_str == "BFS") {
            heuristic_type = HeuristicType::BFS;
        } else if(heuristic_str == "SINGLETON") {
            heuristic_type = HeuristicType::SINGLETON;
        } else if(heuristic_str == "MST") {
            printf("MST heuristic is no longer supported.\n");
            exit(1);
        } else if(heuristic_str == "TSP") {
            heuristic_type = HeuristicType::TSP;
        } else if(heuristic_str == "MAX") {
            heuristic_type = HeuristicType::MAX;
        } else if(heuristic_str == "LAZY") {
            heuristic_type = HeuristicType::LAZY;
        } else {
            throw std::runtime_error("Invalid heuristic type: " + heuristic_str);
        }

        std::string focal_str = parsed_data["focal_method"].get<std::string>();
        FocalMethod focal_method;
        if(focal_str == "SOC") {
            focal_method = FocalMethod::SOC;
        } else if(focal_str == "MOC") {
            focal_method = FocalMethod::MOC;
        } else {
            throw std::runtime_error("Invalid focal method: " + focal_str);
        }

        std::string cell_pruning_str = parsed_data["cell_pruning_method"].get<std::string>();
        CellPruningMethod cell_pruning_method;
        if(cell_pruning_str == "NONE") {
            cell_pruning_method = CellPruningMethod::NONE;
        } else if(cell_pruning_str == "CELL") {
            cell_pruning_method = CellPruningMethod::CELL_DOMINATION;
        } else if(cell_pruning_str == "PATH") {
            cell_pruning_method = CellPruningMethod::PATH_DOMINATION;
        } else if(cell_pruning_str == "CELL_THEN_PATH") {
            cell_pruning_method = CellPruningMethod::CELL_THEN_PATH_DOMINATION;
        } else {
            throw std::runtime_error("Invalid cell pruning method: " + cell_pruning_str);
        }

        Optimizations optimizations;
        optimizations.prune_pivots = parsed_data["prune_pivots"].get<bool>();
        optimizations.run_parallel = parsed_data["run_parallel"].get<bool>();
        optimizations.expand_lowest_cost_agent_only = parsed_data["expand_lowest_cost_agent_only"].get<bool>();
        optimizations.max_pivots_generated = parsed_data["max_pivots_generated"].get<int>();
        optimizations.max_pivots_after_pruning = parsed_data["max_pivots_after_pruning"].get<int>();
        optimizations.parallel_batch_size = parsed_data["parallel_batch_size"].get<int>();

        double centralized_focal_epsilon = parsed_data["centralized_focal_epsilon"].get<double>();
        double centralized_focal_heuristic_weight = parsed_data["centralized_focal_heuristic_weight"].get<double>();
        double centralized_search_time_limit = parsed_data["centralized_search_time_limit"].get<double>();
        double centralized_hard_search_time_limit = parsed_data["centralized_hard_search_time_limit"].get<double>();
        double centralized_astar_weight = parsed_data["centralized_astar_weight"].get<double>();
        bool run_decentralized_search = parsed_data["run_decentralized_search"].get<bool>();
        double decentralized_focal_epsilon = parsed_data["decentralized_focal_epsilon"].get<double>();
        double decentralized_focal_heuristic_weight = parsed_data["decentralized_focal_heuristic_weight"].get<double>();
        double decentralized_search_time_limit = parsed_data["decentralized_search_time_limit"].get<double>();
        double decentralized_hard_search_time_limit = parsed_data["decentralized_hard_search_time_limit"].get<double>();
        double decentralized_astar_weight = parsed_data["decentralized_astar_weight"].get<double>();
        int max_decentralized_searches = parsed_data["max_decentralized_searches"].get<int>();

        return ProblemInput{
            .heuristic_type = heuristic_type,
            .focal_method = focal_method,
            .cell_pruning_method = cell_pruning_method,
            .optimizations = optimizations,
            .centralized_focal_epsilon = centralized_focal_epsilon,
            .centralized_focal_heuristic_weight = centralized_focal_heuristic_weight,
            .centralized_search_time_limit = centralized_search_time_limit,
            .centralized_hard_search_time_limit = centralized_hard_search_time_limit,
            .centralized_astar_weight = centralized_astar_weight,
            .run_decentralized_search = run_decentralized_search,
            .decentralized_focal_epsilon = decentralized_focal_epsilon,
            .decentralized_focal_heuristic_weight = decentralized_focal_heuristic_weight,
            .decentralized_search_time_limit = decentralized_search_time_limit,
            .decentralized_hard_search_time_limit = decentralized_hard_search_time_limit,
            .decentralized_astar_weight = decentralized_astar_weight,
            .max_decentralized_searches = max_decentralized_searches
        };
    }
};


struct SolverConfig {
    HeuristicType heuristic_type;
    FocalMethod focal_method;
    Optimizations optimizations;
    double focal_epsilon;
    double focal_heuristic_weight;
    double search_time_limit;
    double hard_search_time_limit;
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

inline std::vector<std::string> split(std::string s, const std::string& delimiter) {
    std::vector<std::string> tokens;
    size_t pos = 0;
    std::string token;
    while ((pos = s.find(delimiter)) != std::string::npos) {
        token = s.substr(0, pos);
        tokens.push_back(token);
        s.erase(0, pos + delimiter.length());
    }
    tokens.push_back(s);

    return tokens;
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
            int num_agents_required = 1;
            if(task.contains("num_agents_required")) {
                num_agents_required = task["num_agents_required"].get<int>();
            }
            Position task_pos = Position{task_position[0], task_position[1]};
            tasks.push_back(Task(task_id, task_pos, map.get_map_idx(task_pos), num_agents_required));
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
