#pragma once

#include <ilcplex/ilocplex.h>
#include <vector>
#include <iostream>

using namespace std;

int run_mtsp(int num_agents, int num_pivots, const std::vector<std::vector<int>>& cost_matrix) {
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

        // std::vector<std::vector<int>> tours;

        // Solve
        IloCplex cplex(model);
        cplex.setParam(IloCplex::TiLim, 60); // 60 seconds time limit
        cplex.setOut(env.getNullStream());
        cplex.setParam(IloCplex::PreInd, 1); // enable presolve.
        cplex.setParam(IloCplex::MIPEmphasis, 1); // emphasize finding feasible solutions faster.
        cplex.setParam(IloCplex::CutsFactor, 1.0); // adjust cut aggressiveness.
        cplex.setParam(IloCplex::Threads, 4); // use multiple threads.
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
