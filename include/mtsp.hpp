#pragma once

#include "shared.hpp"
#include <ilcplex/ilocplex.h>
#include <vector>
#include <iostream>
#include <chrono>
#include <unordered_set>

using namespace std;

inline double TOTAL_RUNTIME = 0.0;
inline double SOLVER_RUNTIME = 0.0;
inline int TOTAL_CALLS = 0;

inline std::vector<std::vector<int>> get_greedy_solution(const std::vector<std::vector<int>>& cost_matrix, int n, int m) {
    // Greedy: Go to the next cheapest edge that doesn't violate any constraints.
    std::vector<std::vector<int>> initial_paths(m, std::vector<int>());
    for(int i = 0; i < m; i++){
        initial_paths[i].push_back(n); // Start at depot
    }

    std::unordered_set<int> added;

    while(added.size() < n) {
        int cheapest = -1;
        int cheapest_cost = INT_MAX;
        int cheapest_agent = -1;

        for(int i = 0; i < n; i++) {
            if(added.find(i) != added.end()){
                continue;
            }

            for(int j = 0; j < m; j++){
                int curr = initial_paths[j].back();
                int cost;
                if(curr == n){
                    cost = cost_matrix[n + j][i];
                } else {
                    cost = cost_matrix[curr][i];
                }                

                if(cost < cheapest_cost) {
                    cheapest_cost = cost;
                    cheapest = i;
                    cheapest_agent = j;
                }
            }
        }

        initial_paths[cheapest_agent].push_back(cheapest);
        added.insert(cheapest);
    }

    for(int i = 0; i < m; i++){
        initial_paths[i].push_back(n); // Return to depot
    }
    return initial_paths;
}

inline int run_mtsp(int num_agents, int num_pivots, const std::vector<std::vector<int>>& cost_matrix, const std::vector<int>& current_costs) {
    // printf("Num agents: %d, Num pivots: %d\n", num_agents, num_pivots);
    // printf("Cost Matrix:\n");
    // for(int i = 0; i < cost_matrix.size(); i++){
    //     for(int j = 0; j < cost_matrix[i].size(); j++){
    //         printf("%d ", cost_matrix[i][j]);
    //     }
    //     printf("\n");
    // }
    // printf("END\n");

    auto start = std::chrono::high_resolution_clock::now();
    IloEnv env;
    try {
        IloModel model(env);

        int m = num_agents;
        int n = num_pivots;

        // model, x, u, c

        std::vector<std::vector<std::vector<IloBoolVar>>> x(m, std::vector<std::vector<IloBoolVar>>(n + 1, std::vector<IloBoolVar>(n + 1)));
        for(int agent = 0; agent < m; agent++) {
            for(int from = 0; from < n + 1; from++) {
                for(int to = 0; to < n + 1; to++) {
                    if(from != to) {
                        x[agent][from][to] = IloBoolVar(env, ("x_" + to_string(agent) + "_" + to_string(from) + "_" + to_string(to)).c_str());
                    }
                }
            }
        }

        // Constraint 1: Every location is visited exactly once. Done by ensuring every column sums to 1. But only for the N locations (not dummy depots).
        for(int loc = 0; loc < n; loc++) {
            IloExpr expr(env);
            for(int agent = 0; agent < m; agent++) {
                for(int from = 0; from < n + 1; from++) {
                    if(from != loc) {
                        expr += x[agent][from][loc];
                    }
                }
            }

            model.add(expr == 1);
            expr.end();
        }

        // Constraint 2: Continuous path. Leaving loc - arriving at loc = 0 for all loc.
        for(int agent = 0; agent < m; agent++) {
            for(int loc = 0; loc < n; loc++) {
                IloExpr expr(env);
                for(int from = 0; from < n + 1; from++) {
                    if(from != loc) {
                        expr += x[agent][from][loc]; // Going from any other location to loc (including depot).
                    }
                }

                for(int from = 0; from < n + 1; from++) {
                    if(from != loc) {
                        expr -= x[agent][loc][from]; // Going from loc to any other location (including depot).
                    }
                }

                model.add(expr == 0);
                expr.end();
            }
        }

        // Constraint 3: Every agent has at most one tour. Done by ensuring every agent row sums to at most 1.
        for(int agent = 0; agent < m; agent++) {
            IloExpr expr(env);
            for(int loc = 0; loc < n; loc++) {
                expr += x[agent][n][loc];
            }
            model.add(expr <= 1);
            expr.end();
        }

        // Constraint 4: Agents that leave their depot must return. Done by ensuring that sum of an agents row - sum of their column = 0.
        for(int agent = 0; agent < m; agent++) {
            IloExpr expr(env);
            for(int loc = 0; loc < n; loc++) {
                expr += x[agent][n][loc]; // Leaving depot
                expr -= x[agent][loc][n]; // Arriving at depot
            }
            model.add(expr == 0);
            expr.end();
        }

        // Constraint 5: Subtour elimination. Done using MTZ formulation.
        std::vector<std::vector<IloNumVar>> u(m, std::vector<IloNumVar>(n));
        for(int agent = 0; agent < m; agent++) {
            for(int city = 0; city < n; city++) {
                u[agent][city] = IloNumVar(env, 1, n, ILOFLOAT, ("u_" + to_string(agent) + "_" + to_string(city)).c_str());
            }
        }

        for(int agent = 0; agent < m; agent++) {
            for(int i = 0; i < n; i++) {
                for(int j = 0; j < n; j++) {
                    if(i != j) {
                        IloExpr expr(env);
                        expr = u[agent][i] - u[agent][j] + (n) * x[agent][i][j];
                        model.add(expr <= n - 1);
                        expr.end();
                    }
                }
            }
        }

        // Objective function.
        // For makespan we want to minimize the maximum cost of any agent.
        IloNumVar L(env, 0.0, IloInfinity, ILOFLOAT, "L");
        for(int agent = 0; agent < m; agent++) {
            IloExpr agent_cost(env);
            for(int from = 0; from < n + 1; from++) {
                for(int to = 0; to < n; to++) { // Don't add in the "return to depot" cost.
                    if(from != to) {
                        int c_from = (from == n) ? (n + agent) : from; // If from is depot, map to n (dummy depot index in cost matrix)
                        agent_cost += cost_matrix[c_from][to] * x[agent][from][to];
                    }
                }
            }
            model.add(agent_cost + current_costs[agent] <= L);
            agent_cost.end();
        }

        model.add(IloMinimize(env, L));

        // Attach the MIP start to the model
        IloNumVarArray vars(env);
        IloNumArray vals(env);
        std::vector<std::vector<int>> initial_paths = get_greedy_solution(cost_matrix, n, m);

        for(int agent = 0; agent < m; agent++){
            // Don't add if the agent didn't leave its depot.
            if(initial_paths[agent].size() == 2){
                continue;
            }

            for(int i = 0; i < initial_paths[agent].size() - 1; i++) {
                int from = initial_paths[agent][i];
                int to = initial_paths[agent][i + 1];
                vars.add(x[agent][from][to]);
                vals.add(1.0);  // activate this edge
            }
        }

        // Optional: set all other x to 0 (not strictly necessary)
        for(int agent = 0; agent < m; ++agent){
            const std::vector<int>& initial_path = initial_paths[agent];
            for(int from = 0; from < n+1; ++from){
                auto it = std::find(initial_path.begin(), initial_path.end(), from);
                int from_idx = initial_path.size();

                if(it != initial_path.end()){
                    from_idx = std::distance(initial_path.begin(), it);
                }

                for(int to = 0; to < n+1; ++to){
                    if(from == to){
                        continue;
                    }

                    if(from_idx < initial_path.size() - 1 && to == initial_path[from_idx + 1]){
                        continue; // This edge is already added as 1.
                    }

                    vars.add(x[agent][from][to]);
                    vals.add(0.0);
                }
            }
        }

        // Solve
        IloCplex cplex(model);
        cplex.setParam(IloCplex::TiLim, 60); // 60 seconds time limit
        cplex.setOut(env.getNullStream());
        cplex.setParam(IloCplex::PreInd, 1); // enable presolve.
        cplex.setParam(IloCplex::MIPEmphasis, 1); // emphasize finding feasible solutions faster.
        cplex.setParam(IloCplex::CutsFactor, 1.0); // adjust cut aggressiveness.
        cplex.setParam(IloCplex::Threads, 1); // Setting it to use 1 thread reduces overhead.

        cplex.addMIPStart(vars, vals, IloCplex::MIPStartAuto);

        auto end = std::chrono::high_resolution_clock::now();
        auto seconds_taken = std::chrono::duration<double>(end - start).count();
        TOTAL_RUNTIME += seconds_taken;
        TOTAL_CALLS += 1;

        start = std::chrono::high_resolution_clock::now();

        auto solve = cplex.solve();

        end = std::chrono::high_resolution_clock::now();
        seconds_taken = std::chrono::duration<double>(end - start).count();
        SOLVER_RUNTIME += seconds_taken;

        if(solve) {
            // Print out x variable:
            // for(int agent = 0; agent < m; agent++) {
            //     cout << "Route for agent " << agent << ": ";
            //     for(int from = 0; from < n + 1; from++) {
            //         for(int to = 0; to < n + 1; to++) {
            //             if(from == to) continue;
            //             if(cplex.getValue(x[agent][from][to]) > 0.5) {
            //                 cout << from << "->" << to << "(c=" << cost_matrix[(from == n) ? (n + agent) : from][to] << ") ";
            //             }
            //         }
            //     }
            //     cout << endl;
            // }

            double result = cplex.getObjValue();
            env.end();
            return result;

        } else {
            cout << "No solution found." << endl;
            exit(0);
        }

    } catch(IloException& e) {
        cerr << "CPLEX exception: " << e << endl;
        exit(0);
    } catch(...) {
        cerr << "Unknown exception." << endl;
        exit(0);
    }
    env.end();
    printf("How tf did i get there\n");
    exit(0);
}
