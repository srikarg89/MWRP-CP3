#pragma once

#include <ilcplex/ilocplex.h>
#include <vector>
#include <iostream>
#include <chrono>

using namespace std;

double TOTAL_RUNTIME = 0.0;
double SOLVER_RUNTIME = 0.0;

// TODO: Adapt this for multi-agent TSP.
std::vector<int> get_greedy_solution(const std::vector<std::vector<int>>& cost_matrix, int n) {
    // Greedy: Go to the next cheapest edge that doesn't violate any constraints.
    std::vector<int> initial_path;
    initial_path.push_back(n); // Start at depot

    while(initial_path.size() < n + 1) {
        int curr = initial_path.back();

        int cheapest = -1;
        int cheapest_cost = INT_MAX;
        for(int i = 0; i < n; i++) {
            if(std::find(initial_path.begin(), initial_path.end(), i) != initial_path.end()){
                continue;
            }

            if(cost_matrix[curr][i] < cheapest_cost) {
                cheapest_cost = cost_matrix[curr][i];
                cheapest = i;
            }
        }

        initial_path.push_back(cheapest);
    }

    initial_path.push_back(n); // Return to depot
    return initial_path;
}

int run_mtsp(int num_agents, int num_pivots, const std::vector<std::vector<int>>& cost_matrix, std::vector<int> initial_path) {
    auto start = std::chrono::high_resolution_clock::now();
    IloEnv env;
    try {
        IloModel model(env);

        int m = num_agents;
        int n = num_pivots;

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

        // Objective: Minimize total cost. Assume uniform cost of 1 for every edge. Don't add cost for returning back to depot.
        IloNumVar L(env, 0.0, IloInfinity, ILOFLOAT, "L");
        for(int agent = 0; agent < m; agent++) {
            IloExpr agent_cost(env);
            for(int from = 0; from < n + 1; from++) {
                for(int to = 0; to < n + 1; to++) {
                    if(from != to) {
                        int c_from = (from == n) ? (n + agent) : from; // If from is depot, map to n (dummy depot index in cost matrix)
                        agent_cost += cost_matrix[c_from][to] * x[agent][from][to];
                    }
                }
            }
            model.add(agent_cost <= L);
            agent_cost.end();
        }

        model.add(IloMinimize(env, L));


        // Attach the MIP start to the model
        IloNumVarArray vars(env);
        IloNumArray vals(env);
        if(initial_path.size() == 0){
            initial_path = get_greedy_solution(cost_matrix, n);
        } else {
            printf("Using passed in initial path of size %ld\n", initial_path.size());
        }

        for(int i = 0; i < initial_path.size() - 1; i++) {
            int from = initial_path[i];
            int to = initial_path[i + 1];
            vars.add(x[0][from][to]);
            vals.add(1.0);  // activate this edge
        }

        // Optional: set all other x to 0 (not strictly necessary)
        for(int agent = 0; agent < m; ++agent){
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

        // cplex.setParam(IloCplex::HeurFreq, 1);        // run heuristics often
        // cplex.setParam(IloCplex::RINSHeur, 10);       // enable RINS heuristic
        // cplex.setParam(IloCplex::VarSel, 3);          // strong branching variable selection

        cplex.addMIPStart(vars, vals, IloCplex::MIPStartAuto);

        auto end = std::chrono::high_resolution_clock::now();
        auto seconds_taken = std::chrono::duration<double>(end - start).count();
        TOTAL_RUNTIME += seconds_taken;

        start = std::chrono::high_resolution_clock::now();

        if(cplex.solve()) {
            // cout << "Min makespan: " << cplex.getObjValue() << endl;

            // // Print out x variable:
            // for(int agent = 0; agent < m; agent++) {
            //     cout << "Route for agent " << agent << ": ";
            //     for(int from = 0; from < n + 1; from++) {
            //         for(int to = 0; to < n + 1; to++) {
            //             if(from == to) continue;
            //             if(cplex.getValue(x[agent][from][to]) > 0.5) {
            //                 cout << from << "->" << to << " ";
            //             }
            //         }
            //     }
            //     cout << endl;
            // }
            
            end = std::chrono::high_resolution_clock::now();
            seconds_taken = std::chrono::duration<double>(end - start).count();
            SOLVER_RUNTIME += seconds_taken;
            printf("Total MTSP time: %.6f seconds. Solver time: %.6f seconds\n", TOTAL_RUNTIME, SOLVER_RUNTIME);

            return cplex.getObjValue();

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
