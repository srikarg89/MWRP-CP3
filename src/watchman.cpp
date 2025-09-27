#include "watchman.hpp"
#include "shared.hpp"
#include "watchman_utils.hpp"
#include "pathfinding.hpp"
#include <queue>
#include <unordered_map>
#include <iostream>
#include <chrono>
#include <boost/functional/hash.hpp> // For boost::hash_value

double TOTAL_HEURISTIC_TIME = 0.0;
double TOTAL_TSP_SOLVER_TIME = 0.0;
int NUM_SKIPPED = 0;
int MAX_EXISTING_NODES_SIZE = 0;

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

    int get_mst_heuristic(const DisjointGraph& disjoint_graph){
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

    std::tuple<int, std::vector<int>> get_tsp_heuristic(const DisjointGraph& disjoint_graph){
        auto heuristic_start = std::chrono::high_resolution_clock::now();

        // Create distance matrix. Add in a dummy node that connects to agent with cost 0 and all pivots with large cost (U).

        // // Max cost for any edge is map width * map height.
        // // Number of edges is number of pivots squared.
        // // Since U must be greater than sum of all edges, a safe value is width * height * number_of_pivots^2 + 1.
        // int U = los.size() * disjoint_graph.nodes.size() * disjoint_graph.nodes.size() + 1;

        // TSP path length is at most number of nodes * max cost per edge.
        int U = disjoint_graph.max_edge_cost * (disjoint_graph.nodes.size() + 3); // +3 just to be safe.

        // The last node in distjoint_graph.edge_costs is the agent position.
        // We're adding a new dummy node at the end, so now the last row / col will be the dummy node and
        // the second to last row / col will be the agent node.
        std::vector<std::vector<int>> dist = disjoint_graph.edge_costs;
        for(auto& row : dist){
            row.push_back(U);
        }
        dist.push_back(std::vector<int>(disjoint_graph.nodes.size() + 1, U));

        // Dummy <-> Dummy cost is 0. Dummy <-> Agent cost is 0. Dummy <-> Pivot cost is U.
        dist[dist.size() - 1][dist.size() - 1] = 0;
        dist[dist.size() - 1][dist.size() - 2] = 0;
        dist[dist.size() - 2][dist.size() - 1] = 0;

        // Print dist matrix.
        // printf("TSP Distance Matrix:\n");
        // for(int i = 0; i < dist.size(); i++){
        //     for(int j = 0; j < dist[i].size(); j++){
        //         printf("%d ", dist[i][j]);
        //     }
        //     printf("\n");
        // }

        auto solver_start = std::chrono::high_resolution_clock::now();
        auto [ tsp_solution, tsp_path ] = pathfinding::solve_tsp(dist);

        // Reorder the path to remove the dummy node and start at the agent node.
        while(tsp_path[0] != dist.size() - 2){
            std::rotate(tsp_path.begin(), tsp_path.begin() + 1, tsp_path.end());
        }
        if(tsp_path[1] == dist.size() - 1){
            tsp_path.erase(tsp_path.begin() + 1);
        } else if(tsp_path.back() == dist.size() - 1){
            tsp_path.pop_back();
        } else {
            printf("WTF WHERE IS THE PATH\n");
            exit(0);
        }

        std::vector<int> tsp_pivots_path;
        for(int i : tsp_path){
            tsp_pivots_path.push_back(disjoint_graph.nodes[i]);
        }

        int tsp_heuristic = tsp_solution - U; // Subtract out the cost of the dummy -> pivot edge.

        auto end = std::chrono::high_resolution_clock::now();
        auto heuristic_seconds_taken = std::chrono::duration<double>(end - heuristic_start).count();
        auto tsp_solver_seconds_taken = std::chrono::duration<double>(end - solver_start).count();
        TOTAL_HEURISTIC_TIME += heuristic_seconds_taken;
        TOTAL_TSP_SOLVER_TIME += tsp_solver_seconds_taken;

        assert(tsp_heuristic >= 0);
        return std::make_tuple(tsp_heuristic, tsp_pivots_path);
    }

    void prune_graph(DisjointGraph& graph, const Lookup& lookup){
        while(true){
            bool removed = false;
            // printf("Graph nodes size: %ld\n", graph.nodes.size());
            int shortcut_pivot = -1;
            int biggest_shortcut = 0;
            int shortcut_from = -1;
            int shortcut_to = -1;

            for(int i = 0; i < graph.nodes.size() - 1; i++){
                int pivot = graph.nodes[i];
                // printf("Testing pivot %d which is node idx %d\n", pivot, i);
                // BFS out to find distance to every other pivot.
                std::vector<int> distances(graph.nodes.size(), INT_MAX);
                std::vector<int> pred(graph.nodes.size(), -1);
                distances[i] = 0;
                std::priority_queue<std::tuple<int, int>, std::vector<std::tuple<int, int>>, std::greater<>> queue; // cost, node idx
                queue.push(std::make_tuple(0, i));
                while(!queue.empty()){
                    auto [curr_cost, curr_idx] = queue.top();
                    queue.pop();

                    if(distances[curr_idx] < curr_cost){
                        continue;
                    }

                    distances[curr_idx] = curr_cost;

                    for(int neighbor_idx = 0; neighbor_idx < graph.nodes.size(); neighbor_idx++){
                        if(neighbor_idx == curr_idx){
                            continue;
                        }
                        int edge_cost = graph.edge_costs[curr_idx][neighbor_idx];
                        if(edge_cost == INT_MAX){
                            continue;
                        }
                        int new_cost = curr_cost + edge_cost;
                        if(new_cost < distances[neighbor_idx]){
                            distances[neighbor_idx] = new_cost;
                            pred[neighbor_idx] = curr_idx;
                            queue.push(std::make_tuple(new_cost, neighbor_idx));
                        }
                    }
                }

                for(int j = 0; j < graph.nodes.size(); j++){
                    if(i == j || pred[j] == -1 || pred[j] == graph.nodes.size() - 1){
                        continue;
                    }
                    // printf("\tChecking path to pivot %d which is idx %d\n", graph.nodes[j], j);
                    if(pred[j] != i && pred[pred[j]] == i){
                        int shortcut = graph.edge_costs[i][j] - distances[j];
                        // printf("\tFound shortcut between %d and %d that goes through %d. Shortcut amount: %d\n", graph.nodes[i], graph.nodes[j], graph.nodes[pred[j]], shortcut);
                        if(shortcut > biggest_shortcut) {
                            biggest_shortcut = shortcut;
                            shortcut_pivot = pred[j];
                            shortcut_from = i;
                            shortcut_to = j;
                        }
                        // printf("\tFound a faster path from %d to %d that goes through %d\n", i, j, pred[j]);
                        // printf("Pruning pivot: %d\n", graph.nodes[pred[j]]);
                        // graph.nodes.erase(graph.nodes.begin() + pred[j]);
                        // graph.edge_costs.erase(graph.edge_costs.begin() + pred[j]);
                        // for(auto& row : graph.edge_costs){
                        //     row.erase(row.begin() + pred[j]);
                        // }
                        // removed = true;
                        // break;
                    }
                }

                // if(removed){
                //     break;
                // }
            }
            // if(!removed){
            //     break;
            // }
            if(shortcut_pivot == -1){
                break;
            }
            // printf("Pruning pivot: %d which has shortcut: %d in path from %d to %d\n", graph.nodes[shortcut_pivot], biggest_shortcut, graph.nodes[shortcut_from], graph.nodes[shortcut_to]);
            graph.nodes.erase(graph.nodes.begin() + shortcut_pivot);
            graph.edge_costs.erase(graph.edge_costs.begin() + shortcut_pivot);
            for(auto& row : graph.edge_costs){
                row.erase(row.begin() + shortcut_pivot);
            }
        }
    }

    void prune_graph2(DisjointGraph& graph, const Lookup& lookup){
        while(true){
            bool removed = false;
            // printf("Graph nodes size: %ld\n", graph.nodes.size());
            int shortcut_pivot = -1;
            int biggest_shortcut = 0;
            int shortcut_from = -1;
            int shortcut_to = -1;

            int i = graph.nodes.size() - 1;
            int pivot = graph.nodes[i];
            // printf("Testing pivot %d which is node idx %d\n", pivot, i);
            // BFS out to find distance to every other pivot.
            std::vector<int> distances(graph.nodes.size(), INT_MAX);
            std::vector<int> pred(graph.nodes.size(), -1);
            distances[i] = 0;
            std::priority_queue<std::tuple<int, int>, std::vector<std::tuple<int, int>>, std::greater<>> queue; // cost, node idx
            queue.push(std::make_tuple(0, i));
            while(!queue.empty()){
                auto [curr_cost, curr_idx] = queue.top();
                queue.pop();

                if(distances[curr_idx] < curr_cost){
                    continue;
                }

                distances[curr_idx] = curr_cost;

                for(int neighbor_idx = 0; neighbor_idx < graph.nodes.size(); neighbor_idx++){
                    if(neighbor_idx == curr_idx){
                        continue;
                    }
                    int edge_cost = graph.edge_costs[curr_idx][neighbor_idx];
                    if(edge_cost == INT_MAX){
                        continue;
                    }
                    int new_cost = curr_cost + edge_cost;
                    if(new_cost < distances[neighbor_idx]){
                        distances[neighbor_idx] = new_cost;
                        pred[neighbor_idx] = curr_idx;
                        queue.push(std::make_tuple(new_cost, neighbor_idx));
                    }
                }

                for(int j = 0; j < graph.nodes.size(); j++){
                    if(i == j || pred[j] == -1 || pred[j] == graph.nodes.size() - 1){
                        continue;
                    }
                    // printf("\tChecking path to pivot %d which is idx %d\n", graph.nodes[j], j);
                    if(pred[j] != i && pred[pred[j]] == i){
                        int shortcut = graph.edge_costs[i][j] - distances[j];
                        printf("\tFound shortcut between %d and %d that goes through %d. Shortcut amount: %d\n", graph.nodes[i], graph.nodes[j], graph.nodes[pred[j]], shortcut);
                        if(shortcut > biggest_shortcut) {
                            biggest_shortcut = shortcut;
                            shortcut_pivot = pred[j];
                            shortcut_from = i;
                            shortcut_to = j;
                        }
                        // printf("\tFound a faster path from %d to %d that goes through %d\n", i, j, pred[j]);
                        // printf("Pruning pivot: %d\n", graph.nodes[pred[j]]);
                        // graph.nodes.erase(graph.nodes.begin() + pred[j]);
                        // graph.edge_costs.erase(graph.edge_costs.begin() + pred[j]);
                        // for(auto& row : graph.edge_costs){
                        //     row.erase(row.begin() + pred[j]);
                        // }
                        // removed = true;
                        // break;
                    }
                }

                // if(removed){
                //     break;
                // }
            }
            // if(!removed){
            //     break;
            // }
            if(shortcut_pivot == -1){
                break;
            }
            printf("Pruning pivot: %d which has shortcut: %d in path from %d to %d\n", graph.nodes[shortcut_pivot], biggest_shortcut, graph.nodes[shortcut_from], graph.nodes[shortcut_to]);
            graph.nodes.erase(graph.nodes.begin() + shortcut_pivot);
            graph.edge_costs.erase(graph.edge_costs.begin() + shortcut_pivot);
            for(auto& row : graph.edge_costs){
                row.erase(row.begin() + shortcut_pivot);
            }

        }
    }



    int get_heuristic(HeuristicType heuristic_type, const Map& map, std::vector<AgentState> agent_states, const boost::dynamic_bitset<>& seen, const Lookup& lookup){
        switch(heuristic_type){
            case BFS:
                return get_bfs_heuristic();
            case SINGLETON:
                if(agent_states.size() != 1){
                    printf("SINGLETON heuristic only supports single-agent watchman.\n");
                    assert(false);
                    exit(1);
                }
                return get_singleton_heuristic(map.get_map_idx(agent_states[0].pos), seen, lookup.min_dist_to_see);
            case MST: {
                if(agent_states.size() != 1){
                    printf("MST heuristic only supports single-agent watchman.\n");
                    assert(false);
                    exit(1);
                }
                DisjointGraph disjoint_graph_mst = compute_disjoint_graph(lookup, map.get_map_idx(agent_states[0].pos), seen);

                if(disjoint_graph_mst.nodes.size() <= 1){
                    return get_singleton_heuristic(map.get_map_idx(agent_states[0].pos), seen, lookup.min_dist_to_see);
                }
                prune_graph(disjoint_graph_mst, lookup);

                return get_mst_heuristic(disjoint_graph_mst);
            }
            case TSP: {
                if(agent_states.size() != 1){
                    printf("TSP heuristic only supports single-agent watchman.\n");
                    assert(false);
                    exit(1);
                }
                // printf("Starting!\n");
                // Calculate the disjoint graph.
                // printf("\n\nCalculating TSP heuristic for state at %s:\n", agent_states[0].pos.toString().c_str());
                DisjointGraph disjoint_graph_tsp = compute_disjoint_graph(lookup, map.get_map_idx(agent_states[0].pos), seen);
                DisjointGraph original_djg = disjoint_graph_tsp;

                if(disjoint_graph_tsp.nodes.size() <= 1){
                    return get_singleton_heuristic(map.get_map_idx(agent_states[0].pos), seen, lookup.min_dist_to_see);
                }

                DisjointGraph copy_to_prune = disjoint_graph_tsp;
                prune_graph(copy_to_prune, lookup);
                
                // int tsp_heuristic = std::get<0>(get_tsp_heuristic(disjoint_graph_tsp));
                // // printf("Original heuristic with all pivots: %d\n", tsp_heuristic);

                // while(disjoint_graph_tsp.nodes.size() > 1){
                //     int pivot_to_remove = -1;
                //     int best_heuristic_this_iteration = tsp_heuristic;
                //     DisjointGraph best_graph = disjoint_graph_tsp;
                //     for(int i = 0; i < disjoint_graph_tsp.nodes.size() - 1; i++){
                //         DisjointGraph graph_copy = disjoint_graph_tsp;
                //         graph_copy.nodes.erase(graph_copy.nodes.begin() + i);
                //         graph_copy.edge_costs.erase(graph_copy.edge_costs.begin() + i);
                //         for(auto& row : graph_copy.edge_costs){
                //             row.erase(row.begin() + i);
                //         }
                //         int heuristic_with_pivot_removed = std::get<0>(get_tsp_heuristic(graph_copy));
                //         if(heuristic_with_pivot_removed > best_heuristic_this_iteration){
                //             best_heuristic_this_iteration = heuristic_with_pivot_removed;
                //             pivot_to_remove = disjoint_graph_tsp.nodes[i];
                //             best_graph = graph_copy;
                //         }
                //     }
                //     if(pivot_to_remove == -1){
                //         break;
                //     } else {
                //         // printf("Removing pivot %d from disjoint graph. New heuristic: %d\n", pivot_to_remove, best_heuristic_this_iteration);
                //         disjoint_graph_tsp = best_graph;
                //         printf("Removing pivot: %d because it decreases heuristic by %d\n", pivot_to_remove, best_heuristic_this_iteration - tsp_heuristic);
                //         tsp_heuristic = best_heuristic_this_iteration;
                //     }
                // }

                // auto [ original_heuristic, original_path ] = get_tsp_heuristic(original_djg);
                // auto [ prune_heuristic, prune_path ] = get_tsp_heuristic(copy_to_prune);
                // auto [ final_heuristic, final_path ] = get_tsp_heuristic(disjoint_graph_tsp);
                // int ori_idx = 0;
                // int back_idx = original_path.size() - 1;

                // for(int pivot : final_path) {
                //     if(pivot == map.get_map_idx(agent_states[0].pos)) {
                //         continue;
                //     }
                //     while(ori_idx < original_path.size() && original_path[ori_idx] != pivot) {
                //         ori_idx++;
                //     }
                //     while(back_idx >= 0 && original_path[back_idx] != pivot) {
                //         back_idx--;
                //     }
                // }

                // if(prune_heuristic > final_heuristic) {
                //     printf("Pruned heuristic %d is greater than final heuristic %d\n", prune_heuristic, final_heuristic);
                //     printf("Final path: ");
                //     for(int p : final_path) {
                //         printf("%d ", p);
                //     }
                //     printf("\nOriginal path: ");
                //     for(int p : original_path) {
                //         printf("%d ", p);
                //     }
                //     printf("\nPruned path: ");
                //     for(int p : prune_path) {
                //         printf("%d ", p);
                //     }
                //     printf("\n");
                //     printf("Original heuristic: %d, Pruned heuristic: %d, Final heuristic: %d\n", original_heuristic, prune_heuristic, final_heuristic);
                //     assert(false);
                //     exit(0);
                // }

                auto [ prune_heuristic, prune_path ] = get_tsp_heuristic(copy_to_prune);
                int tsp_heuristic = prune_heuristic;


                // if(ori_idx == original_path.size() && back_idx == -1) {
                //     printf("Final path: ");
                //     for(int p : final_path) {
                //         printf("%d ", p);
                //     }
                //     printf("\nOriginal path: ");
                //     for(int p : original_path) {
                //         printf("%d ", p);
                //     }
                //     printf("\nPruned path: ");
                //     for(int p : prune_path) {
                //         printf("%d ", p);
                //     }
                //     printf("\n");
                //     printf("Original heuristic: %d, Pruned heuristic: %d, Final heuristic: %d\n", original_heuristic, prune_heuristic, final_heuristic);
                //     assert(false);
                //     exit(0);
                // }
                

                // if(copy_to_prune.nodes != disjoint_graph_tsp.nodes && get_tsp_heuristic(copy_to_prune) != tsp_heuristic){
                // // if(false){
                //     printf("Original graph nodes: ");
                //     for(int n : original_djg.nodes){
                //         printf("%d ", n);
                //     }
                //     printf("\nOriginal graph edges: \n");
                //     for(auto row : original_djg.edge_costs){
                //         for(int c : row){
                //             printf("%d ", c);
                //         }
                //         printf("\n");
                //     }
                //     get_tsp_heuristic(original_djg);
                //     printf("\nPruned graph nodes: ");
                //     for(int n : copy_to_prune.nodes){
                //         printf("%d ", n);
                //     }
                //     printf("\nPruned graph edges: \n");
                //     for(auto row : copy_to_prune.edge_costs){
                //         for(int c : row){
                //             printf("%d ", c);
                //         }
                //         printf("\n");
                //     }
                //     get_tsp_heuristic(copy_to_prune);
                //     printf("\nElimination chosen graph nodes: ");
                //     for(int n : disjoint_graph_tsp.nodes){
                //         printf("%d ", n);
                //     }
                //     printf("\nElimination graph edges: \n");
                //     for(auto row : disjoint_graph_tsp.edge_costs){
                //         for(int c : row){
                //             printf("%d ", c);
                //         }
                //         printf("\n");
                //     }
                //     get_tsp_heuristic(disjoint_graph_tsp);
                //     printf("\n");
                //     printf("Original heuristic: %d, Pruned heuristic: %d, Final heuristic: %d\n", get_tsp_heuristic(original_djg), get_tsp_heuristic(copy_to_prune), tsp_heuristic);
                //     exit(0);
                // }

                return tsp_heuristic;
            }
            default:
                printf("UNKNOWN HEURISTIC TYPE ???\n");
                assert(false);
                exit(1);
        }
        assert(false);
        return -1;
    }

    std::vector<std::vector<AgentState>> get_possible_moves(const Map& map, const std::vector<AgentState>& agents, MovementType movement){
        std::vector<std::vector<AgentState>> options; 
        for(AgentState agent : agents){
            if(agent.terminated){
                // If any agent is terminated, we don't generate any neighbors.
                options.push_back({agent});
                continue;
            }
            std::vector<AgentState> agent_options;
            std::vector<Position> nbrs = map.get_neighbors(agent.pos, movement);
            // TODO: Jump to frontier.
            for(Position nbr : nbrs){
                AgentState new_agent_state = AgentState{.pos=nbr, .terminated=false};
                agent_options.push_back(new_agent_state);
            }
            agent_options.push_back(AgentState{.pos=agent.pos, .terminated=true}); // Option to terminate.
            options.push_back(agent_options);
        }
        // Now, take the cartesian product of options. Don't include states in which all of the agents terminate.
        std::vector<std::vector<AgentState>> all_moves;
        std::function<void(int, std::vector<AgentState>, bool)> backtrack = [&](int idx, std::vector<AgentState> current, bool has_non_terminated_agent){
            if(idx == options.size()){
                if(has_non_terminated_agent){
                    all_moves.push_back(current);
                }
                return;
            }
            for(AgentState option : options[idx]){
                current.push_back(option);
                backtrack(idx + 1, current, has_non_terminated_agent || !option.terminated);
                current.pop_back();
            }
        };
        backtrack(0, {}, false);
        return all_moves;
    }

    bool is_subset(Position pos, const boost::dynamic_bitset<>& seen, int num_seen, const Node& node){
        if(node.num_seen < num_seen){
            return false;
        }
        return (pos.x == node.agents[0].pos.x) && (pos.y == node.agents[0].pos.y) && ((seen & node.seen) == seen);
    }

    std::string bitset_to_string(const boost::dynamic_bitset<>& seen){
        std::string str = "[";
        for(int i = 0; i < seen.size(); i++){
            str += seen[i] ? "1" : "0";
        }
        str += "]";
        return str;
    }

    // TODO: Fix the cost calculation if we do weighted edges instead of just saying "node.cost + 1".
    std::vector<Node> get_neighbors(Node& node, const Map& map, MovementType movement, const Lookup& lookup, HeuristicType heuristic_type, int last_id_assigned, bool jump_to_frontier, std::unordered_map<std::tuple<std::string, size_t>, int, boost::hash<std::tuple<std::string, size_t>>>& generated_costs){
        std::vector<Node> neighbor_nodes;
        auto neighbors = get_possible_moves(map, node.agents, movement);
        for(const auto& nbr : neighbors){
            boost::dynamic_bitset<> nbr_seen = node.seen;
            // For each neighbor, loop through path to neighbor.
            int new_squares_seen = 0;
            int nbr_cost = node.cost;
            for(const AgentState& agent : nbr){
                new_squares_seen += add_los_to_seen(nbr_seen, lookup.los[map.get_map_idx(agent.pos)], map);
                if(!agent.terminated){
                    nbr_cost += 1;
                }
            }

            int nbr_num_seen = node.num_seen + new_squares_seen;

            std::tuple<std::string, size_t> nbr_key = std::make_tuple(agent_states_to_string(nbr), boost::hash_value(nbr_seen));
            if(generated_costs.find(nbr_key) != generated_costs.end()){
                if(nbr_cost >= generated_costs[nbr_key]){
                    // We've already generated a cheaper version of this node.
                    NUM_SKIPPED += 1;
                    continue;
                } else {
                    generated_costs[nbr_key] = nbr_cost;
                }
            } else {
                generated_costs[nbr_key] = nbr_cost;
            }

            // Calculate heuristic.
            int nbr_heuristic = get_heuristic(heuristic_type, map, nbr, nbr_seen, lookup);
            last_id_assigned += 1;
            neighbor_nodes.push_back(Node(last_id_assigned, nbr, nbr_seen, nbr_cost, nbr_heuristic, nbr_num_seen));
        }
        return neighbor_nodes;
    }

    void write_node_to_file(std::ofstream& file, const Node& node, Lookup& lookup, const Map& map, int parent_id){
        std::unordered_set<int> pivots;
        std::unordered_set<int> watchers;
        std::unordered_set<int> agents;
        for(const AgentState& agent : node.agents){
            agents.insert(map.get_map_idx(agent.pos));
        }

        // DisjointGraph disjoint_graph = compute_disjoint_graph(lookup, agents, node.seen);
        // for(int pivot : disjoint_graph.nodes){
        //     if(agents.find(pivot) != agents.end()){
        //         continue;
        //     }
        //     pivots.insert(pivot);
        //     for(Position watcher : lookup.los[pivot]){
        //         watchers.insert(map.get_map_idx(watcher));
        //     }
        // }

        // file << node.node_id << "," << parent_id << "," << node.agents.size() << ", " << node.cost << "," << node.heuristic << "," << node.f_value << "," << node.num_seen << ",";
        // for(AgentState agent : node.agents){
        //     file << agent.pos.x << "," << agent.pos.y << ",";
        // }
        // file << bitset_to_string(node.seen) << "\n";


        std::string map_list = "";
        for(int i = 0; i < node.seen.size(); i++){
            if(pivots.find(i) != pivots.end()){
                map_list += "3"; // Pivot
            } else if(watchers.find(i) != watchers.end()){
                map_list += "4"; // Watcher
            }
            else if(node.seen[i]){
                map_list += "2"; // Seen
            } else {
                map_list += "0"; // Not seen
            }
        }
        // file << "Node ID, X, Y, Cost, Heuristic, F Value, Num Seen, Map Bitset\n"; // Header
        file << node.node_id << "," << node.agents.size() << ",";
        for(AgentState agent : node.agents){
            file << agent.pos.x << "," << agent.pos.y << ",";
        }
        file << node.cost << "," << node.heuristic << "," << node.f_value << "," << node.num_seen << "," << map_list << "\n";
    }

    // Inputs: Agent starting positions, LOS type, map.
    // TODO: Add in radius for LOS.
    // Output: Optimal path.
    std::vector<std::vector<Position>> run_watchman(std::vector<Position> starts, LOSType los, const Map& map, MovementType movement, HeuristicType heuristic_type, bool jump_to_frontier){
        Lookup lookup;
        precompute_lookup(lookup, los, map, movement, heuristic_type, starts[0]);

        // Create initial seen bitset.
        boost::dynamic_bitset<> start_seen(map.x_size * map.y_size);
        int num_start_seen = 0;
        for(int map_idx = 0; map_idx < map.x_size * map.y_size; map_idx++){
            if(map.check_obstacle(map.get_pos_from_map_idx(map_idx))){
                start_seen[map_idx] = 1;
                num_start_seen += 1;
            }
        }
        std::vector<AgentState> start_agent_states;
        for(Position start : starts){
            num_start_seen += add_los_to_seen(start_seen, lookup.los[map.get_map_idx(start)], map);
            start_agent_states.push_back(AgentState{.pos=start, .terminated=false});
        }

        // Initialize search data structures.
        std::priority_queue<Node, std::vector<Node>, std::greater<Node>> queue;
        std::unordered_map<int, int> pred_lookup;
        std::unordered_map<int, std::vector<AgentState>> id_lookup;
        std::unordered_set<std::tuple<std::string, size_t>, boost::hash<std::tuple<std::string, size_t>>> visited_nodes; // (map_idx, seen bitset hash)
        std::unordered_map<std::tuple<std::string, size_t>, int, boost::hash<std::tuple<std::string, size_t>>> generated_costs; // (map_idx, seen bitset hash)

        std::ofstream debug_file;
        debug_file.open("watchman_debug.csv");
        debug_file << "Node ID, X, Y, Cost, Heuristic, F Value, Num Seen, Seen Bitset\n"; // Header

        // Setup the starting variable. Mark all obstacles as seen.
        int num_obstacles = num_start_seen;
        int num_free = map.x_size * map.y_size - num_obstacles;


        int start_heuristic = get_heuristic(heuristic_type, map, start_agent_states, start_seen, lookup);
        printf("Start heuristic: %d\n", start_heuristic);
        queue.push(Node(/* id = */ 0, start_agent_states, start_seen, /* cost = */ 0, start_heuristic, num_start_seen));

        std::vector<Node> expanded_nodes;

        int num_expanded = 0;
        int num_generated = 0;
        int last_id_assigned = 0;
        int max_new_squares_seen = 0;
        int num_skipped = 0;
        int num_skipped_dom = 0;

        auto start_time = std::chrono::high_resolution_clock::now();

        std::vector<std::vector<Position>> path;

        while(!queue.empty()){
            Node curr = queue.top();
            queue.pop();

            size_t seen_hash = boost::hash_value(curr.seen);
            std::tuple<std::string, size_t> visited_key = std::make_tuple(agent_states_to_string(curr.agents), seen_hash);
            if(visited_nodes.find(visited_key) != visited_nodes.end()){
                // Already visited this node.
                num_skipped += 1;
                continue;
            }

            expanded_nodes.push_back(curr);

            visited_nodes.insert(visited_key);
            id_lookup[curr.node_id] = curr.agents;

            write_node_to_file(debug_file, curr, lookup, map, pred_lookup[curr.node_id]);

            // debug_file.close(); exit(0);

            max_new_squares_seen = std::max(max_new_squares_seen, curr.num_seen - num_obstacles);
            num_expanded += 1;
            if(num_expanded % 1000 == 0){
            // if(num_expanded % 1 == 0){
                printf("Expanded %d nodes. Loc: %s, cost: %d, heuristic: %d, num new seen: %d / %d, max new squares seen: %d\n", num_expanded, agent_states_to_string(curr.agents).c_str(), curr.cost, curr.heuristic, (curr.num_seen - num_obstacles), num_free, max_new_squares_seen);
                printf("\tF value: %d. Next node's f value: %d\n", curr.f_value, (queue.empty() ? -1 : queue.top().f_value));
            }
            printf("Expanding node %d. Node ID: %d, Loc: %s, cost: %d, heuristic: %d, num seen: %d\n", num_expanded, curr.node_id, agent_states_to_string(curr.agents).c_str(), curr.cost, curr.heuristic, curr.num_seen);

            if(curr.num_seen == map.x_size * map.y_size){
                printf("Goal condition met!\n");
                auto end_time = std::chrono::high_resolution_clock::now();
                auto seconds_taken = std::chrono::duration<double>(end_time - start_time).count();
                printf("Search time taken: %.3f seconds\n", seconds_taken);

                path.push_back(agent_states_to_positions(curr.agents));
                int curr_id = curr.node_id;
                while(curr_id != 0){
                    curr_id = pred_lookup[curr_id];
                    path.push_back(agent_states_to_positions(id_lookup[curr_id]));
                }
                std::reverse(path.begin(), path.end());
                break;;
            }

            std::vector<Node> neighbors = get_neighbors(curr, map, movement, lookup, heuristic_type, last_id_assigned, jump_to_frontier, generated_costs);
            num_generated += neighbors.size();
            if(neighbors.size() > 0){
                last_id_assigned = neighbors.back().node_id;
            }
            for(Node& nbr : neighbors){
                // printf("\tGenerated neighbor. Node ID: %d, Loc: %s, cost: %d, heuristic: %d, num seen: %d\n", nbr.node_id, nbr.pos.toString().c_str(), nbr.cost, nbr.heuristic, nbr.num_seen);
                pred_lookup[nbr.node_id] = curr.node_id;
                queue.push(nbr);
            }
        }

        printf("Total nodes expanded: %d\n", num_expanded);
        printf("Total expansions skipped: %d\n", num_skipped);
        printf("Total expansions skipped by domination check: %d\n", num_skipped_dom);
        printf("Total generations skipped: %d\n", NUM_SKIPPED);
        printf("Total nodes generated: %d\n", num_generated);
        printf("Total heuristic time: %.3f seconds\n", TOTAL_HEURISTIC_TIME);
        printf("Total TSP solver time: %.3f seconds\n", TOTAL_TSP_SOLVER_TIME);
        printf("Total TSP brute force: %.6f seconds\n", TOTAL_TSP_BRUTE_FORCE_TIME);
        printf("Total TSP concorde time: %.6f seconds\n", TOTAL_TSP_CONCORDE_TIME);
        printf("Total brute force calls: %d\n", TOTAL_TSP_BRUTE_FORCE_CALLS);
        printf("Total concorde calls: %d\n", TOTAL_TSP_CONCORDE_CALLS);
        debug_file.close();

        return path;
    }

}