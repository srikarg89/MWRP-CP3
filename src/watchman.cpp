#include "watchman.hpp"
#include "shared.hpp"
#include "watchman_utils.hpp"
#include "pathfinding.hpp"
#include <queue>
#include <unordered_map>
#include <iostream>
#include <boost/functional/hash.hpp> // For boost::hash_value

namespace watchman {

    int get_bfs_heuristic(){
        return 1;
    }

    int get_singleton_heuristic(int node_map_idx, const boost::dynamic_bitset<>& seen, const std::vector<std::vector<int>>& min_dist_to_see){
        int heuristic = 0;
        for(int i = 0; i < seen.size(); i++){
            if(!seen[i]) {
                heuristic = std::max(heuristic, min_dist_to_see[node_map_idx][i]);
            }
        }
        return heuristic;
    }

    int get_mst_heuristic(const Lookup& lookup, int node_map_idx, const boost::dynamic_bitset<>& seen){
        // Calculate the disjoint graph.
        DisjointGraph disjoint_graph = compute_disjoint_graph(lookup, node_map_idx, seen);

        // Calculate minimum spanning tree.
        std::priority_queue<std::tuple<int, int, int>> edge_set; // 
        int mst_heuristic = 0;
        std::vector<int> distances(disjoint_graph.nodes.size(), INT_MAX);
        std::vector<bool> added(disjoint_graph.nodes.size(), false);

        // printf("\nMST Calculation:\n");

        for(int i = 0; i < disjoint_graph.nodes.size(); i++){
            // Find the min cost node.
            int next_node_to_add = 0;
            int min_cost = INT_MAX;
            for(int j = 0; j < distances.size(); j++){
                if(added[j]){
                    continue;
                }
                if(distances[j] < min_cost){
                    next_node_to_add = j;
                    min_cost = distances[j];
                }
            }

            if(i > 0){
                mst_heuristic += min_cost;
            }

            for(int j = 0; j < distances.size(); j++){
                distances[j] = std::min(distances[j], disjoint_graph.edge_costs[next_node_to_add][j]);
            }

            added[next_node_to_add] = true;
        }

        // printf("MST Heuristic: %d\n", mst_heuristic);
        return mst_heuristic;
    }

    int get_heuristic(HeuristicType heuristic_type, int node_map_idx, const boost::dynamic_bitset<>& seen, const Lookup& lookup){
        switch(heuristic_type){
            case BFS:
                return get_bfs_heuristic();
            case SINGLETON:
                return get_singleton_heuristic(node_map_idx, seen, lookup.min_dist_to_see);
            case MST:
                return get_mst_heuristic(lookup, node_map_idx, seen);
            default:
                printf("UNKNOWN HEURISTIC TYPE ???\n");
                assert(false);
        }
        assert(false);
        return -1;
    }

    // TODO: Fix the cost calculation if we do weighted edges instead of just saying "node.cost + 1".
    std::vector<Node> get_neighbors(Node& node, const Map& map, MovementType movement, const Lookup& lookup, HeuristicType heuristic_type, int last_id_assigned){
        std::vector<Position> neighbors = map.get_neighbors(node.pos, movement);
        std::vector<Node> neighbor_nodes;
        for(Position nbr : neighbors){
            boost::dynamic_bitset<> nbr_seen = node.seen;
            int new_squares_seen = add_los_to_seen(nbr_seen, lookup.los[map.get_map_idx(nbr)], map);
            int nbr_heuristic = get_heuristic(heuristic_type, map.get_map_idx(nbr), nbr_seen, lookup);
            last_id_assigned += 1;
            neighbor_nodes.push_back(Node(last_id_assigned, nbr, nbr_seen, node.cost + 1, nbr_heuristic, node.num_seen + new_squares_seen));
        }
        return neighbor_nodes;
    }

    // Inputs: Agent starting position, LOS type, map.
    // TODO: Add in radius for LOS.
    // Output: Optimal path.
    std::vector<Position> run_watchman(Position start, LOSType los, const Map& map, MovementType movement, HeuristicType heuristic_type){
        Lookup lookup;
        precompute_lookup(lookup, los, map, movement, heuristic_type);

        // Initialize search data structures.
        std::priority_queue<Node, std::vector<Node>, std::greater<Node>> queue;
        std::unordered_map<int, int> pred_lookup;
        std::unordered_map<int, Position> id_lookup;
        std::unordered_set<std::tuple<int, size_t>, boost::hash<std::tuple<int, size_t>>> visited_nodes; // (map_idx, seen bitset hash)

        // Setup the starting variable. Mark all obstacles as seen.
        boost::dynamic_bitset<> start_seen(map.x_size * map.y_size);
        int num_start_seen = 0;
        for(int map_idx = 0; map_idx < map.x_size * map.y_size; map_idx++){
            if(map.check_obstacle(map.get_pos_from_map_idx(map_idx))){
                start_seen[map_idx] = 1;
                num_start_seen += 1;
            }
        }
        int num_obstacles = num_start_seen;
        int num_free = map.x_size * map.y_size - num_obstacles;
        num_start_seen += add_los_to_seen(start_seen, lookup.los[map.get_map_idx(start)], map);
        printf("Start idx: %d\n", map.get_map_idx(start));
        int start_heuristic = get_heuristic(heuristic_type, map.get_map_idx(start), start_seen, lookup);
        queue.push(Node(/* id = */ 0, start, start_seen, /* cost = */ 0, start_heuristic, num_start_seen));

        int num_expanded = 0;
        int last_id_assigned = 0;
        int max_new_squares_seen = 0;
        int num_skipped = 0;

        std::vector<Position> path;

        while(!queue.empty()){
            Node curr = queue.top();
            queue.pop();

            size_t seen_hash = boost::hash_value(curr.seen);
            std::tuple<int, size_t> visited_key = std::make_tuple(map.get_map_idx(curr.pos), seen_hash);
            if(visited_nodes.find(visited_key) != visited_nodes.end()){
                // Already visited this node.
                num_skipped += 1;
                continue;
            }

            visited_nodes.insert(visited_key);
            id_lookup[curr.node_id] = curr.pos;

            max_new_squares_seen = std::max(max_new_squares_seen, curr.num_seen - num_obstacles);
            num_expanded += 1;
            if(num_expanded % 1000 == 0){
            // if(num_expanded % 1 == 0){
                printf("Expanded %d nodes. Loc: %s, cost: %d, heuristic: %d, num new seen: %d / %d, max new squares seen: %d\n", num_expanded, curr.pos.toString().c_str(), curr.cost, curr.heuristic, (curr.num_seen - num_obstacles), num_free, max_new_squares_seen);
                printf("\tF value: %d. Next node's f value: %d\n", curr.f_value, (queue.empty() ? -1 : queue.top().f_value));
            }
            // printf("Expanding node %d. Loc: %s, cost: %d, heuristic: %d, num seen: %d\n", num_expanded, curr.pos.toString().c_str(), curr.cost, curr.heuristic, curr.num_seen);

            if(curr.num_seen == map.x_size * map.y_size){
                printf("Goal condition met!\n");
                path.push_back(curr.pos);
                int curr_id = curr.node_id;
                while(curr_id != 0){
                    curr_id = pred_lookup[curr_id];
                    path.push_back(id_lookup[curr_id]);
                }
                std::reverse(path.begin(), path.end());
                break;
            }

            std::vector<Node> neighbors = get_neighbors(curr, map, movement, lookup, heuristic_type, last_id_assigned);
            if(neighbors.size() > 0){
                last_id_assigned = neighbors.back().node_id;
            }
            for(Node& nbr : neighbors){
                pred_lookup[nbr.node_id] = curr.node_id;
                queue.push(nbr);
            }
        }

        printf("Total nodes expanded: %d\n", num_expanded);
        printf("Total nodes skipped: %d\n", num_skipped);

        return path;

    }

}