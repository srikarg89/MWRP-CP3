#pragma once

#include "shared.hpp"
#include "mtsp.hpp"

#include <unordered_map>
#include <unordered_set>
#include <queue>
#include <cstdlib>
#include <climits>
#include <cassert>
#include <algorithm>
#include <chrono>

// Used to include Concorde TSP solver C code.
#define new new_
#define class class_

extern "C" {
    #include "concorde.h"
}

#undef new
#undef class

double TOTAL_MTSP_TIME = 0.0;
double TOTAL_TSP_BRUTE_FORCE_TIME = 0.0;
double TOTAL_TSP_CONCORDE_TIME = 0.0;
int TOTAL_MTSP_CALLS = 0;
int TOTAL_TSP_BRUTE_FORCE_CALLS = 0;
int TOTAL_TSP_CONCORDE_CALLS = 0;


namespace pathfinding {
    std::tuple<std::vector<int>, std::vector<int>> get_bfs_distances_and_preds(std::vector<Position> starts, const Map& map){
        std::vector<int> distances(map.x_size * map.y_size, INT_MAX);
        std::vector<int> preds(map.x_size * map.y_size, INT_MAX);
        std::queue<int> queue;

        for(Position start : starts){
            int start_map_idx = map.get_map_idx(start);
            queue.push(start_map_idx);
            distances[start_map_idx] = 0;
            preds[start_map_idx] = -1;
        }

        while(!queue.empty()){
            int curr = queue.front();
            queue.pop();
            for(int neighbor_map_idx : map.neighbors[curr]){
                if(distances[neighbor_map_idx] != INT_MAX){
                    continue;
                }
                distances[neighbor_map_idx] = distances[curr] + 1;
                preds[neighbor_map_idx] = curr;
                queue.push(neighbor_map_idx);
            }
        }

        return std::make_tuple(distances, preds);
    }

    // Brute-Force TSP Solver for small number of nodes.
    std::tuple<int, std::vector<int>> solve_tsp_brute_force(const std::vector<std::vector<int>>& dist) {
        std::vector<int> perm;
        for(int i = 0; i < dist.size(); i++) {
            perm.push_back(i);
        }

        int best = INT_MAX;
        std::vector<int> best_perm;

        // Loop through every permutation.
        do {
            // Calculate cost of this permutation.
            int cost = 0;
            for(int i = 0; i < perm.size(); i++){
                int from = perm[i];
                int to = perm[(i + 1) % perm.size()]; // wrap around
                cost += dist[from][to];
            }

            if(cost < best) {
                best_perm = perm;
            }
            best = std::min(best, cost);
        } while (next_permutation(perm.begin(), perm.end()));

        return std::make_tuple(best, best_perm);
    }

    // TSP Solver using Concorde solver functions
    std::tuple<int, std::vector<int>> solve_tsp_concorde(const std::vector<std::vector<int>>& dist) {
        double optval;
        int success, foundtour, hit_timebound = 0;

        // Setup adjacency matrix.
        int ncount = dist.size();
        int ecount = (ncount * (ncount - 1)) / 2;
        int *elist = new int[ecount * 2];
        int *elen = new int[ecount];
        int edge = 0;
        int edgeWeight = 0;
        int idx = 0;
        for (int i = 0; i < ncount; i++) {
            for (int j = i + 1; j < ncount; j++) {
                elist[idx*2] = i;
                elist[idx*2 + 1] = j;
                elen[idx] = dist[i][j];
                idx += 1;
            }
        }

        // Random state to seed the solver.
        CCrandstate rstate;
        CCutil_sprand(rand(), &rstate);

        int *out_tour = CC_SAFE_MALLOC(ncount, int);
        char *name = CCtsp_problabel(" ");

        // Redirect stdout and stderr to /dev/null to suppress Concorde output.
        int devnull = open("/dev/null", O_WRONLY);
        int saved_stdout = dup(STDOUT_FILENO);
        dup2(devnull, STDOUT_FILENO);
        close(devnull);

        // Solve TSP function.
        CCtsp_solve_sparse(ncount, ecount, elist, elen, /* in_tour = */ NULL, out_tour, /* in_val = */ NULL, &optval, &success, &foundtour, name, NULL, &hit_timebound, /* silent = */ 1, &rstate);

        // Restore stdout
        dup2(saved_stdout, STDOUT_FILENO);
        close(saved_stdout);

        // Check return code.
        if(success != 1 || foundtour != 1) {
            fprintf(stderr, "TSP solver failed to find a solution: %d, %d\n", success, foundtour);
            exit(1);
        }

        std::vector<int> tour;
        for(int i = 0; i < ncount; i++){
            tour.push_back(out_tour[i]);
        }

        // Free allocated variables.
        CC_IFFREE(elist, int);
        CC_IFFREE(elen, int);
        CC_IFFREE(out_tour, int);
        CC_IFFREE(name, char);

        // return tour;
        // return optval;
        return std::make_tuple((int)optval, tour);
    }

    // TSP Solver function that chooses between brute-force and Concorde based on number of nodes.
    std::tuple<int, std::vector<int>> solve_tsp(const std::vector<std::vector<int>>& dist){
        auto start = std::chrono::high_resolution_clock::now();

        if(dist.size() <= 4){
            // printf("Dist size: %ld. Solving brute force\n", dist.size());
            TOTAL_TSP_BRUTE_FORCE_CALLS += 1;
            auto ret = solve_tsp_brute_force(dist);
            auto end = std::chrono::high_resolution_clock::now();
            auto seconds_taken = std::chrono::duration<double>(end - start).count();
            TOTAL_TSP_BRUTE_FORCE_TIME += seconds_taken;

            // if(std::get<0>(ret) != mtsp_solution){
            //     printf("Brute force TSP solution %d != MTSP solution %d\n", std::get<0>(ret), mtsp_solution);
            //     exit(1);
            // }
            return ret;
        }
        else{
            // printf("Dist size: %ld. Solving Concorde\n", dist.size());
            TOTAL_TSP_CONCORDE_CALLS += 1;
            auto ret = solve_tsp_concorde(dist);
            auto end = std::chrono::high_resolution_clock::now();
            auto seconds_taken = std::chrono::duration<double>(end - start).count();
            TOTAL_TSP_CONCORDE_TIME += seconds_taken;
            // printf("Concorde TSP Solver. Dist: %d and took %.6f seconds\n", dist.size(), seconds_taken);

            // TOTAL_MTSP_CALLS += 1;
            // printf("Calls: %d\n", TOTAL_MTSP_CALLS);
            // start = std::chrono::high_resolution_clock::now();
            // int mtsp_solution = run_mtsp(1, dist.size() - 1, dist); // Empty initial path to use greedy.
            // end = std::chrono::high_resolution_clock::now();
            // seconds_taken = std::chrono::duration<double>(end - start).count();
            // TOTAL_MTSP_TIME += seconds_taken;

            // if(std::get<0>(ret) != mtsp_solution){
            //     printf("Concorde TSP solution %d != MTSP solution %d\n", std::get<0>(ret), mtsp_solution);
            //     printf("Concorde path: ");
            //     for(int i : std::get<1>(ret)){
            //         printf("%d ", i);
            //     }
            //     printf("\n");
            //     exit(1);
            // }
            return ret;
        }
    }

}