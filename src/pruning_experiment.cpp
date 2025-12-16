#include <iostream>
#include <chrono>
#include <algorithm>
#include <ctime>
#include "search.hpp"
#include "environment.hpp"
#include "heirarchical_search.hpp"

Map get_map(std::string map_filename) {
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

    Map map(map_filename, x_size, y_size, occupancy, FOUR_WAY_MOVEMENT, BRES_LOS);
    return std::move(map);
}

std::vector<Position> get_random_agent_starts_border_raw(const Map& map, int num_agents, int border_width) {
    std::vector<Position> options;
    for(int x = 0; x < map.x_size; x++) {
        for(int y = 0; y < map.y_size; y++) {
            if(x < border_width || x >= map.x_size - border_width || y < border_width || y >= map.y_size - border_width) {
                Position pos = {x, y};
                if(!map.check_obstacle(pos)) {
                    options.push_back(pos);
                }
            }
        }
    }

    std::random_shuffle(options.begin(), options.end());

    std::vector<Position> agent_starts;
    for(int i = 0; i < num_agents; i++) {
        agent_starts.push_back(options[i]);
    }

    return agent_starts;
}

std::vector<Position> get_random_agent_starts_border_floodfill(const Map& map, int num_agents) {
    std::vector<Position> options;
    std::vector<Position> edge_cells;
    for(int x = 0; x < map.x_size; x++) {
        for(int y = 0; y < map.y_size; y++) {
            if(x < 1 || x >= map.x_size - 1 || y < 1 || y >= map.y_size - 1) {
                Position pos = {x, y};
                if(map.check_obstacle(pos)) {
                    edge_cells.push_back(pos);
                } else {
                    options.push_back(pos);
                }
            }
        }
    }

    // BFS flood fill from obstacles on border to find allowed border cells.
    std::vector<std::vector<bool>> visited(map.x_size, std::vector<bool>(map.y_size, false));
    std::queue<Position> queue;

    for(Position pos : edge_cells) {
        queue.push(pos);
        visited[pos.x][pos.y] = true;
    }

    while(!queue.empty()) {
        Position curr = queue.front();
        queue.pop();

        int dX[] = {-1, 1, 0, 0};
        int dY[] = {0, 0, -1, 1};

        for(int i = 0; i < 4; i++) {
            Position neighbor = curr.add(dX[i], dY[i]);
            if(neighbor.x < 0 || neighbor.x >= map.x_size || neighbor.y < 0 || neighbor.y >= map.y_size) {
                continue;
            }
            if(visited[neighbor.x][neighbor.y]) {
                continue;
            }
            if(map.check_obstacle(neighbor)) {
                visited[neighbor.x][neighbor.y] = true;
                queue.push(neighbor);
            } else {
                options.push_back(neighbor);
            }
        }
    }

    std::random_shuffle(options.begin(), options.end());

    std::vector<Position> agent_starts;
    for(int i = 0; i < num_agents; i++) {
        agent_starts.push_back(options[i]);
    }

    return agent_starts;
}

std::vector<Position> get_random_agent_starts(const Map& map, int num_agents) {
    std::vector<Position> agent_starts;
    for(int i = 0; i < num_agents; i++) {
        while(true){
            int x = rand() % map.x_size;
            int y = rand() % map.y_size;
            Position pos = {x, y};
            if(map.check_obstacle(pos)) {
                continue;
            }
            agent_starts.push_back(pos);
            break;
        }
    }

    return agent_starts;
}

void compute_lookup_los_and_apsp(Lookup& lookup, const Map& map) {
    // Precompute the LOS Lookup and the All Pairs Shortest Path (APSP)
    printf("Precomputing lookup!\n");
    auto start_time = std::chrono::high_resolution_clock::now();
    for(int map_idx = 0; map_idx < map.num_squares; map_idx++){
        lookup.watchers.push_back({});
        lookup.watchers_set.push_back({});
    }

    for(int map_idx = 0; map_idx < map.num_squares; map_idx++){
        Position pos = map.get_pos_from_map_idx(map_idx);
        if(map.check_obstacle(pos)){
            lookup.los.push_back({});
            std::vector<int> infinite_distances(map.num_squares, INT_MAX);
            lookup.apsp.push_back(infinite_distances);
            lookup.apsp_paths.push_back(infinite_distances);
        } else {
            switch(map.los_type){
                case FOUR_WAY_LOS:
                    lookup.los.push_back(four_way_LOS(pos, map));
                    break;
                case EIGHT_WAY_LOS:
                    lookup.los.push_back(eight_way_LOS(pos, map));
                    break;
                case BRES_LOS:
                    lookup.los.push_back(bresLOS(pos, map));
                    break;                        
            }

            for(Position los_pos : lookup.los.back()){
                int los_map_idx = map.get_map_idx(los_pos);
                lookup.watchers[los_map_idx].push_back(pos);
                lookup.watchers_set[los_map_idx].insert(map_idx);
            }

            auto [distances, preds] = get_bfs_distances_and_preds({pos}, map);
            lookup.apsp.push_back(distances);
            lookup.apsp_paths.push_back(preds);
        }
    }

    auto end_time = std::chrono::high_resolution_clock::now();
    std::chrono::duration<double> duration = end_time - start_time;
    printf("LOS and APSP precomputation time: %.6f seconds\n", duration.count());
}

std::pair<std::vector<bool>, double> prune(const Map& map, std::vector<Position> agent_starts, CellPruningMethod cell_pruning_method) {
    Lookup lookup;
    compute_lookup_los_and_apsp(lookup, map);

    auto start_time = std::chrono::high_resolution_clock::now();

    if(cell_pruning_method == CellPruningMethod::CELL_DOMINATION){
        // Find squares whose watchers are dominated by another square.
        lookup.strictly_easier = calculate_square_dominance(map, lookup.watchers, lookup.watchers_set);
    } else if(cell_pruning_method == CellPruningMethod::PATH_DOMINATION || cell_pruning_method == CellPruningMethod::CELL_THEN_PATH_DOMINATION){
        std::vector<bool> cells_to_ignore = std::vector<bool>(map.num_squares, false);
        if(cell_pruning_method == CellPruningMethod::CELL_THEN_PATH_DOMINATION){
            cells_to_ignore = calculate_square_dominance(map, lookup.watchers, lookup.watchers_set);
        };

        // Find path dominance
        std::vector<boost::dynamic_bitset<>> combined_path_dominations = std::vector<boost::dynamic_bitset<>>(map.num_squares, boost::dynamic_bitset<>(map.num_squares, 0));
        for(int i = 0; i < map.num_squares; i++){
            combined_path_dominations[i].set(); // Initialize all to 1.
        }

        std::vector<boost::dynamic_bitset<>> watchers_bitsets(map.num_squares, boost::dynamic_bitset<>(map.num_squares, 0));
        for(int i = 0; i < map.num_squares; i++){
            for(Position watcher_pos : lookup.watchers[i]){
                int watcher_map_idx = map.get_map_idx(watcher_pos);
                watchers_bitsets[i][watcher_map_idx] = 1;
            }
            watchers_bitsets[i][i] = 1; // Any cell can see itself.
        }

        std::vector<boost::dynamic_bitset<>> agent_path_dominations = get_path_dominations_matrix(agent_starts, map, watchers_bitsets, cells_to_ignore);
        lookup.strictly_easier = path_dominations_to_strictly_easier(map, agent_path_dominations);

        if(cell_pruning_method == CellPruningMethod::CELL_THEN_PATH_DOMINATION){
            for(int i = 0; i < map.num_squares; i++){
                lookup.strictly_easier[i] = lookup.strictly_easier[i] | cells_to_ignore[i];
            }
        }
    } else {
        throw std::runtime_error("Unknown cell pruning method!");
    }

    auto end_time = std::chrono::high_resolution_clock::now();
    std::chrono::duration<double> duration = end_time - start_time;
    printf("Strictly easier precomputation time: %.6f seconds\n", duration.count());
    return std::make_pair(lookup.strictly_easier, duration.count());
}

int get_num_free_cells_initially(const Map& map, std::vector<Position> agent_starts, Lookup& lookup) {
    int count = 0;
    for(int i = 0; i < map.num_squares; i++) {
        if(map.check_obstacle(map.get_pos_from_map_idx(i))) {
            continue;
        }
        bool in_los = false;
        for(Position pos : agent_starts) {
            int pos_map_idx = map.get_map_idx(pos);
            if(lookup.watchers_set[i].find(pos_map_idx) != lookup.watchers_set[i].end()) {
                in_los = true;
                break;
            }
        }
        if(in_los) {
            continue;
        }
        count += 1;
    }
    return count;
}

int get_num_free_cells_starting_config(const Map& map, Lookup& lookup, std::vector<Position> agent_starts) {
    int count = 0;
    for(int i = 0; i < map.num_squares; i++) {
        if(map.check_obstacle(map.get_pos_from_map_idx(i))) {
            continue;
        }
        bool is_watcher = false;
        for(Position pos : agent_starts) {
            if(lookup.watchers_set[i].find(map.get_map_idx(pos)) != lookup.watchers_set[i].end()) {
                is_watcher = true;
                break;
            }
        }
        if(is_watcher) {
            continue;
        }
        count += 1;
    }
    return count;
}

int get_num_free_cells_left(const std::vector<bool>& strictly_easier, const Map& map, Lookup& lookup, std::vector<Position> agent_starts) {
    int count = 0;
    for(int i = 0; i < map.num_squares; i++) {
        if(map.check_obstacle(map.get_pos_from_map_idx(i))) {
            continue;
        }
        if(strictly_easier[i]) {
            continue;
        }
        bool is_watcher = false;
        for(Position pos : agent_starts) {
            if(lookup.watchers_set[i].find(map.get_map_idx(pos)) != lookup.watchers_set[i].end()) {
                is_watcher = true;
                break;
            }
        }
        if(is_watcher) {
            continue;
        }
        count += 1;
    }
    return count;
}

int get_w(const std::vector<bool>& strictly_easier, const Map& map, Lookup& lookup, std::vector<Position> agent_starts) {
    std::vector<bool> valid_watchers(map.num_squares, false);
    for(int i = 0; i < map.num_squares; i++) {
        if(map.check_obstacle(map.get_pos_from_map_idx(i))) {
            continue;
        }
        if(strictly_easier[i]) {
            continue;
        }
        bool is_watcher = false;
        for(Position pos : agent_starts) {
            if(lookup.watchers_set[i].find(map.get_map_idx(pos)) != lookup.watchers_set[i].end()) {
                is_watcher = true;
                break;
            }
        }
        if(is_watcher) {
            continue;
        }

        // Unseen cell
        for(Position watcher_pos : lookup.watchers[i]) {
            int watcher_map_idx = map.get_map_idx(watcher_pos);
            valid_watchers[watcher_map_idx] = true;
        }
    }

    int count = 0;
    for(bool valid : valid_watchers) {
        if(valid) {
            count += 1;
        }
    }
    return count;
}

void write_to_file(std::ofstream& file, std::string map_name, int num_agents, CellPruningMethod method, int num_free_cells, int num_free_cells_left, double runtime) {
    std::string method_string = "";
    if(method == CellPruningMethod::CELL_DOMINATION) {
        method_string = "CELL";
    } else if(method == CellPruningMethod::PATH_DOMINATION) {
        method_string = "PATH";
    } else if(method == CellPruningMethod::CELL_THEN_PATH_DOMINATION) {
        method_string = "CELL_THEN_PATH";
    }

    double runtime_ms = runtime * 1000.0;

    printf("Writing to file!\n");
    file << map_name << ", " << method_string << ", " << num_agents << ", " << num_free_cells_left << ", " << num_free_cells << ", " << runtime_ms << "\n";
}


int main() {
    std::ofstream temp;
    temp.open("pruning_experiment.csv");
    temp.close();

    // std::vector<std::string> map_filenames = {"../maps/maze-32-32-2.map", "../maps/huge/ht_chantry.map", "../maps/lak202.map", "../maps/room-64-64-8.map"};
    // std::vector<std::string> map_filenames = {"../maps/maze-32-32-2.map", "../maps/room-64-64-8.map"};
    // std::vector<std::string> map_filenames = {"../maps/random-20-20-80.map", "../maps/room-24-24-4.map", "../maps/mc-forest.map"};
    std::vector<std::string> map_filenames = {"../maps/random-20-20-80.map", "../maps/maze-32-32-2.map", "../maps/mc-forest.map"};

    for(std::string map_filename : map_filenames) {
        Map map = get_map(map_filename);
        Lookup lookup;
        compute_lookup_los_and_apsp(lookup, map);

        std::vector<int> us = {0, 0, 0};
        std::vector<double> times = {0.0, 0.0, 0.0};
        std::ofstream file;
        file.open("pruning_experiment.csv", std::ios::app);
        file << "Map, Method, Exp number, U, Initial Free Cells, Average Time (ms)\n";
        file.close();

        // for(int num_agents : {1, 2, 3, 4}){
        for(int num_agents : {1}){
            for(int i = 0; i < 10; i++){
                printf("\n\nExperiment %d on map %s with %d agents\n", i, map_filename.c_str(), num_agents);
                std::vector<Position> agent_starts;
                if(map_filename == "../maps/maze-32-32-2.map" || map_filename == "../maps/room-64-64-8.map" || map_filename == "../maps/room-24-24-4.map" || map_filename == "../maps/random-20-20-80.map") {
                    agent_starts = get_random_agent_starts_border_raw(map, num_agents, 2);
                } else {
                    agent_starts = get_random_agent_starts_border_floodfill(map, num_agents);
                }


                auto result = prune(map, agent_starts, CellPruningMethod::CELL_DOMINATION);
                double cell_u = get_num_free_cells_left(result.first, map, lookup, agent_starts);
                double cell_t = result.second;
                us[0] += cell_u;
                times[0] += cell_t;

                result = prune(map, agent_starts, CellPruningMethod::PATH_DOMINATION);
                double path_u = get_num_free_cells_left(result.first, map, lookup, agent_starts);
                double path_t = result.second;
                us[1] += path_u;
                times[1] += path_t;

                result = prune(map, agent_starts, CellPruningMethod::CELL_THEN_PATH_DOMINATION);
                double cell_then_path_u = get_num_free_cells_left(result.first, map, lookup, agent_starts);
                double cell_then_path_t = result.second;
                us[2] += cell_then_path_u;
                times[2] += cell_then_path_t;

                printf("CD: U=%.2f, Time=%.2fms | PD: U=%.2f, Time=%.2fms | CPD: U=%.2f, Time=%.2fms\n", cell_u, cell_t * 1000.0, path_u, path_t * 1000.0, cell_then_path_u, cell_then_path_t * 1000.0);

                file.open("pruning_experiment.csv", std::ios::app);
                file << map_filename << ", " << i << ", CELL, " << cell_u << ", " << get_num_free_cells_initially(map, agent_starts, lookup) << ", " << cell_t * 1000.0 << "\n";
                file << map_filename << ", " << i << ", PATH, " << path_u << ", " << get_num_free_cells_initially(map, agent_starts, lookup) << ", " << path_t * 1000.0 << "\n";
                file << map_filename << ", " << i << ", CELL_THEN_PATH, " << cell_then_path_u << ", " << get_num_free_cells_initially(map, agent_starts, lookup) << ", " << cell_then_path_t * 1000.0 << "\n";
                file.close();
            }
        }
        printf("Average U for Cell Domination on map %s: %.2f\n", map_filename.c_str(), (double)us[0] / 10.0);
        printf("Average U for Path Domination on map %s: %.2f\n", map_filename.c_str(), (double)us[1] / 10.0);
        printf("Average U for Cell Then Path Domination on map %s: %.2f\n", map_filename.c_str(), (double)us[2] / 10.0);
        printf("Average time for Cell Domination on map %s: %.2f\n", map_filename.c_str(), times[0] / 10.0);
        printf("Average time for Path Domination on map %s: %.2f\n", map_filename.c_str(), times[1] / 10.0);
        printf("Average time for Cell Then Path Domination on map %s: %.2f\n", map_filename.c_str(), times[2] / 10.0);
    }

    // std::ofstream temp;
    // temp.open("pruning_experiment.csv");
    // temp.close();

    // for(int i = 0; i < 50; i++){
    //     // std::ofstream file;
    //     // file.open("pruning_experiment.csv", std::ios::app);

    //     for(std::string map_filename : map_filenames) {
    //         Map map = get_map(map_filename);
    //         std::vector<Position> agent_starts = get_random_agent_starts(map, 1);
    //         Lookup lookup;
    //         compute_lookup_los_and_apsp(lookup, map);

    //         printf("\n\nRunning Cell Domination on map %s\n", map_filename.c_str());
    //         auto result = prune(map, agent_starts, CellPruningMethod::CELL_DOMINATION);
    //         printf("New unseen cells: %d, new W: %d\n", get_num_free_cells_left(result.first, map, lookup, agent_starts), get_w(result.first, map, lookup, agent_starts));
    //         // write_to_file(file, map_filename, agent_starts.size(), CellPruningMethod::CELL_DOMINATION, get_num_free_cells_initially(map), get_num_free_cells_left(result.first, map, lookup, agent_starts), result.second);

    //         printf("\n\nRunning Path Domination on map %s with %ld agents\n", map_filename.c_str(), agent_starts.size());
    //         result = prune(map, agent_starts, CellPruningMethod::PATH_DOMINATION);
    //         printf("New unseen cells: %d, new W: %d\n", get_num_free_cells_left(result.first, map, lookup, agent_starts), get_w(result.first, map, lookup, agent_starts));
    //         // write_to_file(file, map_filename, agent_starts.size(), CellPruningMethod::PATH_DOMINATION, get_num_free_cells_initially(map), get_num_free_cells_left(result.first, map, lookup, agent_starts), result.second);

    //         printf("\n\nRunning Cell Then Path Domination on map %s with %ld agents\n", map_filename.c_str(), agent_starts.size());
    //         result = prune(map, agent_starts, CellPruningMethod::CELL_THEN_PATH_DOMINATION);
    //         printf("New unseen cells: %d, new W: %d\n", get_num_free_cells_left(result.first, map, lookup, agent_starts), get_w(result.first, map, lookup, agent_starts));
    //         // write_to_file(file, map_filename, agent_starts.size(), CellPruningMethod::CELL_THEN_PATH_DOMINATION, get_num_free_cells_initially(map), get_num_free_cells_left(result.first, map, lookup, agent_starts), result.second);
    //     }        

    //     // std::string map_filename = "../maps/huge/ht_chantry.map";
    //     // Map map = get_map(map_filename);
    //     // Lookup lookup;
    //     // compute_lookup_los_and_apsp(lookup, map);

    //     // for(int num_agents : {2, 3, 4, 5, 6, 7, 8}) {
    //     //     std::vector<Position> agent_starts = get_random_agent_starts(map, num_agents);

    //     //     printf("\n\nRunning Path Domination on map %s with %ld agents\n", map_filename.c_str(), agent_starts.size());
    //     //     auto result = prune(map, agent_starts, CellPruningMethod::PATH_DOMINATION);
    //     //     // write_to_file(file, map_filename, agent_starts.size(), CellPruningMethod::PATH_DOMINATION, get_num_free_cells_initially(map), get_num_free_cells_left(result.first, map, lookup, agent_starts), result.second);

    //     //     printf("\n\nRunning Cell Then Path Domination on map %s with %ld agents\n", map_filename.c_str(), agent_starts.size());
    //     //     result = prune(map, agent_starts, CellPruningMethod::CELL_THEN_PATH_DOMINATION);
    //     //     // write_to_file(file, map_filename, agent_starts.size(), CellPruningMethod::CELL_THEN_PATH_DOMINATION, get_num_free_cells_initially(map), get_num_free_cells_left(result.first, map, lookup, agent_starts), result.second);
    //     // }

    //     // file.close();
    // }

    return 0;
}
