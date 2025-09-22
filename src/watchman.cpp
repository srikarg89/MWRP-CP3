#include "shared.hpp"
#include "watchman.hpp"
#include "pathfinding.hpp"
#include <queue>
#include <unordered_map>
#include <iostream>
#include <boost/functional/hash.hpp> // For boost::hash_value

namespace watchman {
    // Returns the number of new squares marked as seen.
    int add_los_to_seen(boost::dynamic_bitset<>& seen, const std::vector<Position>& los, const Map& map){
        int count = 0;
        for(Position pos : los){
            int map_idx = map.get_map_idx(pos);
            if(!seen[map_idx]){
                count += 1;
            }
            seen[map_idx] = 1;
        }
        return count;
    }

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
        // printf("\nComputation for node idx: %d\n", node_map_idx);
        // printf("Disjoint Graph Nodes:\n");
        // for(int node : disjoint_graph.nodes){
        //     printf("\t%d\n", node);
        // }
        // printf("Disjoint Graph Edges:\n");
        // for(int i = 0; i < disjoint_graph.nodes.size(); i++){
        //     for(int j = 0; j < disjoint_graph.nodes.size(); j++){
        //         printf("\t(%d, %d) with cost %d\n", disjoint_graph.nodes[i], disjoint_graph.nodes[j], disjoint_graph.edge_costs[i][j]);
        //     }
        // }

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

            // printf("Popping: %d with cost %d\n", disjoint_graph.nodes[next_node_to_add], min_cost);

            if(i > 0){
                mst_heuristic += min_cost;
                // printf("Adding min cost to mst heuristic: %d\n", min_cost);
            }

            for(int j = 0; j < distances.size(); j++){
                distances[j] = std::min(distances[j], disjoint_graph.edge_costs[next_node_to_add][j]);
            }

            added[next_node_to_add] = true;
        }

        // printf("MST Heuristic: %d\n", mst_heuristic);
        return mst_heuristic;

        // assert(false);
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

    void precompute_lookup(Lookup& lookup, LOSType los, const Map& map, MovementType movement, HeuristicType heuristic_type){
        // Precompute the LOS Lookup and the All Pairs Shortest Path (APSP)
        printf("Running watchman method!\n");
        std::vector<int> sorted_los_order;
        for(int map_idx = 0; map_idx < map.x_size * map.y_size; map_idx++){
            Position pos = map.get_pos_from_map_idx(map_idx);
            if(map.check_obstacle(pos)){
                lookup.los.push_back({});
                std::vector<int> infinite_distances(map.x_size * map.y_size, INT_MAX);
                lookup.apsp.push_back(infinite_distances);
            } else {
                switch(los){
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

                std::vector<int> distances = pathfinding::get_bfs_distances({pos}, movement, map);
                lookup.apsp.push_back(distances);
                sorted_los_order.push_back(map_idx);
            }
            // printf("Position: %s\n\t%s\n", pos.toString().c_str(), pos_array_to_string(lookup.los.back()).c_str());
        }

        printf("\n\n");

        std::sort(sorted_los_order.begin(), sorted_los_order.end(), [lookup](int a, int b){
            return lookup.los[a].size() < lookup.los[b].size();
        });
        lookup.sorted_los_order = sorted_los_order;

        // Precompute the Singleton heuristic helper lookup table.
        if(heuristic_type == HeuristicType::SINGLETON){
            for(int map_idx = 0; map_idx < map.x_size * map.y_size; map_idx++){
                std::vector<int> min_dists(map.x_size * map.y_size, INT_MAX);
                lookup.min_dist_to_see.push_back(min_dists);
            }

            for(int g_map_idx = 0; g_map_idx < map.x_size * map.y_size; g_map_idx++){
                const std::vector<Position>& los = lookup.los[g_map_idx];
                for(Position los_pos : los){
                    int los_idx = map.get_map_idx(los_pos);
                    for(int s_map_idx = 0; s_map_idx < map.x_size * map.y_size; s_map_idx++){
                        lookup.min_dist_to_see[s_map_idx][g_map_idx] = std::min(lookup.min_dist_to_see[s_map_idx][g_map_idx], lookup.apsp[s_map_idx][los_idx]);
                    }
                }
            }
        }

        // Precompute the distance between pivots in G_DLC
        if(heuristic_type == HeuristicType::MST || heuristic_type == HeuristicType::TSP){
            for(int map_idx = 0; map_idx < map.x_size * map.y_size; map_idx++){
                std::vector<int> min_dists(map.x_size * map.y_size, INT_MAX);
                lookup.pivot_pivot_dists.push_back(min_dists);
            }

            for(int pivot_idx = 0; pivot_idx < map.x_size * map.y_size; pivot_idx++){
                if(map.check_obstacle(map.get_pos_from_map_idx(pivot_idx))){
                    std::vector<int> infinite_distances(map.x_size * map.y_size, INT_MAX);
                    lookup.pivot_cell_dists.push_back(infinite_distances);
                    lookup.pivot_pivot_dists.push_back(infinite_distances);
                    continue;
                }

                // For each pivot, we want to compute the distances from the pivot component to every other location.
                // Then, we can use that data to calculate the min dstance from the pivot component to any other pivot component.
                std::vector<int> pivot_cell_dists = pathfinding::get_bfs_distances(lookup.los[pivot_idx], movement, map);
                lookup.pivot_cell_dists.push_back(pivot_cell_dists);

                for(int cell_idx = 0; cell_idx < map.x_size * map.y_size; cell_idx++){
                    for(Position los_pos : lookup.los[cell_idx]){
                        int los_map_idx = map.get_map_idx(los_pos);
                        lookup.pivot_pivot_dists[pivot_idx][cell_idx] = std::min(lookup.pivot_pivot_dists[pivot_idx][cell_idx], pivot_cell_dists[los_map_idx]);
                    }
                }
            }

            // Verified using Figure 3b / 3d in https://cdn.aaai.org/ojs/6668/6668-40-9897-1-10-20200521.pdf

            // printf("C <-> F: %d / %d / %d / %d / %d / %d / %d / %d, D <-> C: %d, D <-> F: %d\n",
            //     lookup.pivot_pivot_dists[map.get_map_idx({3, 0})][map.get_map_idx({1, 4})],
            //     lookup.pivot_pivot_dists[map.get_map_idx({1, 4})][map.get_map_idx({3, 0})],
            //     lookup.pivot_pivot_dists[map.get_map_idx({3, 0})][map.get_map_idx({2, 4})],
            //     lookup.pivot_pivot_dists[map.get_map_idx({2, 4})][map.get_map_idx({3, 0})],
            //     lookup.pivot_pivot_dists[map.get_map_idx({2, 0})][map.get_map_idx({1, 4})],
            //     lookup.pivot_pivot_dists[map.get_map_idx({1, 4})][map.get_map_idx({2, 0})],
            //     lookup.pivot_pivot_dists[map.get_map_idx({2, 0})][map.get_map_idx({2, 4})],
            //     lookup.pivot_pivot_dists[map.get_map_idx({2, 4})][map.get_map_idx({2, 0})],
            //     lookup.pivot_cell_dists[map.get_map_idx({3, 0})][map.get_map_idx({1, 2})],
            //     lookup.pivot_cell_dists[map.get_map_idx({1, 4})][map.get_map_idx({1, 2})]
            // );

            // printf("B <-> next to D: %d\n", lookup.pivot_pivot_dists[map.get_map_idx({2, 0})][map.get_map_idx({2, 0})]);

            // for (int i = 0; i < lookup.pivot_pivot_dists.size(); ++i) {
            //     // Iterate through columns in the current row
            //     for (int j = 0; j < lookup.pivot_pivot_dists.size(); ++j) {
            //         int dist = lookup.pivot_pivot_dists[i][j];
            //         if(dist == INT_MAX){
            //             dist = -1;
            //         }
            //         std::cout << dist << "\t"; // Print element and a tab for spacing
            //     }
            //     std::cout << std::endl; // Move to the next line after printing a row
            // }
        }
    }

    DisjointGraph compute_disjoint_graph(const Lookup& lookup, int agent_map_idx, const boost::dynamic_bitset<>& seen){
        // Step 1: Get all the nodes: Agent position, pivots, watchers. We already processed the distances between pivot components, so don't need to add in the watchers.
        std::vector<int> pivots;
        // std::vector<DisjointGraphEdge> edges;

        for(int potential_pivot : lookup.sorted_los_order){
            // printf("Processing potential pivot: %d\n", potential_pivot);
            if(seen[potential_pivot]){
                // printf("\tSkipping cuz already seen\n");
                continue;
            }

            // We can test if a pivot is valid by checking the pivot <-> pivot distance. If this is 0, then the pivots share a watcher, and thus this pivot is invalid.
            // Technically still O(N^2) but now only checking against pivots instead of every cell, which should be much faster.
            bool valid = true;
            for(int existing_pivot : pivots){
                if(lookup.pivot_pivot_dists[existing_pivot][potential_pivot] == 0){
                    valid = false;
                    // printf("\tSkipping cuz intersects with existing pivot: %d\n", existing_pivot);
                    break;
                }
            }
            if(!valid){
                continue;
            }

            pivots.push_back(potential_pivot);
        }

        std::vector<std::vector<int>> edge_costs;
        std::vector<int> agent_costs;
        for(int p1 : pivots){
            // Add outgoing edges for each pivot.
            std::vector<int> pivot_outgoing_costs;
            for(int p2 : pivots){
                pivot_outgoing_costs.push_back(lookup.pivot_pivot_dists[p1][p2]);
            }
            pivot_outgoing_costs.push_back(lookup.pivot_cell_dists[p1][agent_map_idx]);
            agent_costs.push_back(lookup.pivot_cell_dists[p1][agent_map_idx]);
            edge_costs.push_back(pivot_outgoing_costs);
        }

        // Add agent position to the list of nodes.
        std::vector<int> nodes = pivots;
        nodes.push_back(agent_map_idx);
        agent_costs.push_back(0);

        // Add outgoing edges for the agent.
        edge_costs.push_back(agent_costs);


        // Nodes = pivots + agent position.
        return DisjointGraph {
            .nodes=nodes,
            .edge_costs=edge_costs
        };
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
                int curr_id = curr.node_id;
                printf("\tPosition: %s\n", curr.pos.toString().c_str());
                while(curr_id != 0){
                    curr_id = pred_lookup[curr_id];
                    printf("\tPosition: %s\n", id_lookup[curr_id].toString().c_str());
                }
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

        return {};

    }

}