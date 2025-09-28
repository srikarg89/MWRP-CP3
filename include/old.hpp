#include <ilcplex/ilocplex.h>
#include <vector>
#include <iostream>

using namespace std;

int test_mtsp() {
    IloEnv env;
    try {
        IloModel model(env);

        int n = 5;
        int m = 2;

        vector<vector<int>> cost_matrix = {
            {0,2,9,10,7},
            {2,0,6,4,3},
            {9,6,0,8,5},
            {10,4,8,0,6},
            {7,3,5,6,0},
            {0,0,0,0,0},
            {0,0,0,0,0},
        };

        // std::vector<std::vector<std::vector<IloBoolVar>>> x(m, std::vector<std::vector<IloBoolVar>>(n + 1, std::vector<IloBoolVar>(n + 1)));

        // // Constraint 1: Every location is visited exactly once. Done by ensuring every column sums to 1. But only for the N locations (not dummy depots).
        // for(int loc = 0; loc < n; loc++) {
        //     IloExpr expr(env);
        //     for(int agent = 0; agent < m; agent++) {
        //         for(int from = 0; from < n + 1; from++) {
        //             if(from != loc) {
        //                 expr += x[agent][from][loc];
        //             }
        //         }
        //     }

        //     model.add(expr == 1);
        //     expr.end();
        // }

        // // Constraint 2: Continuous path. Leaving loc - arriving at loc = 0 for all loc.


        // // Constraint 3: Every agent has at most one tour. Done by ensuring every agent row sums to at most 1.
        // for(int agent = 0; agent < m; agent++) {
        //     IloExpr expr(env);
        //     for(int loc = 0; loc < n; loc++) {
        //         expr += x[n + agent][loc];
        //     }
        //     model.add(expr <= 1);
        //     expr.end();
        // }

        // // Constraint 4: Agents that leave their depot must return. Done by ensuring that sum of an agents row - sum of their column = 0.
        // for(int agent = 0; agent < m; agent++) {
        //     IloExpr expr(env);
        //     for(int loc = 0; loc < n; loc++) {
        //         expr += x[n + agent][loc]; // Leaving depot
        //         expr -= x[loc][n + agent]; // Arriving at depot
        //     }
        //     model.add(expr == 0);
        //     expr.end();
        // }

        // // Objective: Minimize total cost. Assume uniform cost of 1 for every edge. Don't add cost for returning back to depot.
        // IloExpr total_cost(env);
        // for(int from = 0; from < m + n; from++) {
        //     for(int to = 0; to < n; to++) {
        //         if(from != to) {
        //             total_cost += x[from][to];
        //         }
        //     }
        // }

        // model.add(IloMinimize(env, total_cost));

        // Decision variables: m + n rows, n columns. Last m rows are dummy depots.
        std::vector<std::vector<IloBoolVar>> x(m + n, std::vector<IloBoolVar>(m + n));
        for(int i = 0; i < m + n; i++) {
            for(int j = 0; j < m + n; j++) {
                if(i != j) {
                    x[i][j] = IloBoolVar(env, ("x_" + to_string(i) + "_" + to_string(j)).c_str());
                }
            }
        }
        
        // Assignment variables
        std::vector<std::vector<IloBoolVar>> a(m, std::vector<IloBoolVar>(n));
        for(int i = 0; i < m; i++) {
            for(int j = 0; j < n; j++) {
                a[i][j] = IloBoolVar(env, ("a_" + to_string(i) + "_" + to_string(j)).c_str());
            }
        }


        std::cout << "GOT HERE" << std::endl;

        // Constraint 1: Every location is visited exactly once. Done by ensuring every column sums to 1. But only for the N locations (not dummy depots).
        for(int loc = 0; loc < n; loc++) {
            IloExpr expr(env);
            for(int from = 0; from < m + n; from++) {
                if(from != loc){
                    expr += x[from][loc];
                }
            }
            model.add(expr == 1);
            expr.end();
        }
        std::cout << "GOT HERE" << std::endl;

        // Constraint 2: Continuous path. Leaving loc - arriving at loc = 0 for all loc.
        for(int loc = 0; loc < n; loc++) {
            IloExpr expr(env);
            IloExpr assignment_expr(env);
            for(int from = 0; from < m + n; from++) {
                if(from != loc) {
                    expr += x[loc][from]; // Leaving loc
                    expr -= x[from][loc]; // Arriving at loc
                }
            }
            model.add(expr == 0);
            expr.end();
        }
        std::cout << "GOT HERE" << std::endl;

        // Constraint 3: Every agent has at most one tour. Done by ensuring every agent row sums to at most 1.
        for(int agent = 0; agent < m; agent++) {
            IloExpr expr(env);
            for(int loc = 0; loc < n; loc++) {
                expr += x[n + agent][loc];
            }
            model.add(expr <= 1);
            expr.end();
        }

        // Constraint 4: Agents that leave their depot must return. Done by ensuring that sum of an agents row - sum of their column = 0.
        for(int agent = 0; agent < m; agent++) {
            IloExpr expr(env);
            for(int loc = 0; loc < n; loc++) {
                expr += x[n + agent][loc]; // Leaving depot
                expr -= x[loc][n + agent]; // Arriving at depot
            }
            model.add(expr == 0);
            expr.end();
        }
        std::cout << "GOT HERE" << std::endl;

        // Constraint 5: Each location is only visited by the agent it is assigned to.
        for(int loc = 0; loc < n; loc++) {
            IloExpr expr(env);
            for(int agent = 0; agent < m; agent++) {
                expr += a[agent][loc]; // Arriving at loc from agent depot
            }
            model.add(expr == 1);
            expr.end();
        }

        std::cout << "HI" << std::endl;

        for(int from = 0; from < n; from++) {
            for(int to = 0; to < n; to++) {
                if(from != to) {
                    for(int agent = 0; agent < m; agent++) {
                        // IloExpr expr(env);
                        // expr += a[agent][from] * a[agent][to] - x[from][to];
                        // model.add(expr == 0);
                        // expr.end();
                        model.add(a[agent][from] - a[agent][to] <= 1 - x[from][to]);
                        model.add(a[agent][to] - a[agent][from] <= 1 - x[from][to]);
                    }
                }
            }
        }
        std::cout << "GOT HERE" << std::endl;

        // Objective function.
        IloNumVar L(env, 0.0, IloInfinity, ILOFLOAT, "L");
        for(int agent = 0; agent < m; agent++) {
            IloExpr agent_cost(env);
            for(int from = 0; from < n; from++) {
                for(int to = 0; to < n; to++) {
                    if(from != to) {
                        agent_cost += x[from][to] * a[agent][to] * cost_matrix[from][to];
                    }
                }
            }
            // Add in cost from depot to first location.
            for(int to = 0; to < n; to++) {
                agent_cost += x[n + agent][to] * cost_matrix[n + agent][to];
            }

            model.add(agent_cost <= L);
            agent_cost.end();
        }

        model.add(IloMinimize(env, L));

        // int n = 5; // number of customers
        // int m = 2; // number of salesmen
        // vector<vector<int>> C = {
        //     {0,2,9,10,7},
        //     {2,0,6,4,3},
        //     {9,6,0,8,5},
        //     {10,4,8,0,6},
        //     {7,3,5,6,0}
        // };
        // int n_prime = n; // no dummy depots in this example

        // std::vector<int> agent_current_costs(m, 0); // assuming all agents start with 0 cost. TODO: Change this.

        // IloModel model(env);

        // // decision variables: s[k][i][j]
        // vector<vector<vector<IloBoolVar>>> s(m,
        //     vector<vector<IloBoolVar>>(n_prime, vector<IloBoolVar>(n_prime)));
        // for(int k=0;k<m;++k)
        //     for(int i=0;i<n_prime;++i)
        //         for(int j=0;j<n_prime;++j)
        //             if(i!=j)
        //                 s[k][i][j] = IloBoolVar(env, ("s_"+to_string(k)+"_"+to_string(i)+"_"+to_string(j)).c_str());
        
        // // MTZ variables
        // vector<IloNumVar> t(n_prime);
        // for(int i=0;i<n_prime;++i)
        //     t[i] = IloNumVar(env, 0, n_prime-1, ILOFLOAT, ("t_"+to_string(i)).c_str());

        // // Makespan variable that we are trying to optimize.
        // IloNumVar z(env, 0.0, IloInfinity, ILOFLOAT, "z");

        // // 1) Each pivot visited exactly once (N constraints).
        // for(int j = 0; j < n_prime; j++) {
        //     IloExpr expr(env);
        //     // Each pivot should only be "reached" once.
        //     for(int k = 0; k < m; k++){
        //         for(int i = 0; i < n_prime; i++){
        //             if(i != j){
        //                 expr += s[k][i][j];
        //             }
        //         }
        //     }
        //     model.add(expr == 1);
        //     expr.end();
        // }

        // // TODO: Each pivot is left exactly once??

        // // 2) Flow conservation per salesman
        // for(int k = 0; k < m; k++) {
        //     for(int v = 0; v < n_prime; v++) {
        //         IloExpr out(env), in(env);
        //         for(int j = 0; j < n_prime; j++){
        //             if(j != v){
        //                 out += s[k][v][j];
        //             }
        //         }
        //         for(int i = 0; i < n_prime; i++) {
        //             if(i != v) {
        //                 in += s[k][i][v];
        //             }
        //         }
        //         model.add(out - in == 0); // adjust for depot if needed
        //         out.end(); in.end();
        //     }
        // }

        // // 3) MTZ subtour elimination to preserve order.
        // for(int i=0;i<n_prime;++i) {
        //     for(int j=0;j<n_prime;++j) {
        //         if(i!=j) {
        //             IloExpr xij(env);
        //             for(int k=0;k<m;++k) xij += s[k][i][j];
        //             model.add(t[i] - t[j] + n_prime * xij <= n_prime - 1);
        //             xij.end();
        //         }
        //     }
        // }

        // // 4) Route length constraints
        // for(int k = 0; k < m; k++) {
        //     IloExpr agent_cost_k(env);
        //     for(int i = 0; i < n_prime; i++){
        //         for(int j = 0; j < n_prime; j++){
        //             if (i != j){
        //                 costk += C[i][j] * s[k][i][j];
        //             }
        //         }
        //     }
        //     cost_k += agent_current_costs;
        //     model.add(costk <= z);
        //     costk.end();
        // }

        // // Objective: minimize makespan
        // model.add(IloMinimize(env, z));

        std::cout << "GOT HERE" << std::endl;

        // Solve
        IloCplex cplex(model);
        cplex.setParam(IloCplex::TiLim, 60); // 60 seconds time limit
        if(cplex.solve()) {
            cout << "Min makespan: " << cplex.getObjValue() << endl;
            for(int k = 0; k < m; k++){
                printf("Assignment for agent %d: ", k);
                for(int j = 0; j < n; j++){
                    if(cplex.getValue(a[k][j]) > 0.5) {
                        printf("%d ", j);
                    }
                }
                printf("\n");
            }

            // Print out x variable:
            for(int i = 0; i < m + n; i++) {
                for(int j = 0; j < m + n; j++) {
                    if(i != j && cplex.getValue(x[i][j]) > 0.5) {
                        printf("Path exists between %d and %d\n", i, j);
                    }
                }
            }

            // for(int k=0;k<m;++k) {
            //     cout << "Route for salesman " << k << ": ";
            //     for(int i=0;i<n_prime;++i)
            //         for(int j=0;j<n_prime;++j)
            //             if(i!=j && cplex.getValue(s[k][i][j]) > 0.5)
            //                 cout << i << "->" << j << " ";
            //     cout << endl;
            // }
        } else {
            cout << "No solution found." << endl;
        }

    } catch(IloException& e) {
        cerr << "CPLEX exception: " << e << endl;
    } catch(...) {
        cerr << "Unknown exception." << endl;
    }
    env.end();
    return 0;
}
