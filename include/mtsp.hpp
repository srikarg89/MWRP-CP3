#pragma once

#include "shared.hpp"
#include <ilcplex/ilocplex.h>
#include <vector>
#include <iostream>
#include <chrono>
#include <unordered_set>

using namespace std;

inline std::vector<std::vector<int>> get_greedy_solution(const std::vector<std::vector<int>>& cost_matrix, int n, int m, const std::vector<int>& num_required_visits) {
    // Greedy: Go to the next cheapest edge that doesn't violate any constraints.
    std::vector<std::vector<int>> initial_paths(m, std::vector<int>());
    for(int i = 0; i < m; i++){
        initial_paths[i].push_back(n); // Start at depot
    }

    std::vector<std::unordered_set<int>> agents_visited(n, std::unordered_set<int>());

    while(true) {
        int cheapest = -1;
        int cheapest_cost = INT_MAX;
        int cheapest_agent = -1;

        bool added = false;
        for(int i = 0; i < n; i++) {
            if(agents_visited[i].size() == num_required_visits[i]){
                continue;
            }

            for(int j = 0; j < m; j++){
                // Agent's already visited this location.
                if(agents_visited[i].find(j) != agents_visited[i].end()){
                    continue;
                }

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
            added = true;
        }

        if(!added){
            break;
        }

        initial_paths[cheapest_agent].push_back(cheapest);
        agents_visited[cheapest].insert(cheapest_agent);
    }

    for(int i = 0; i < m; i++){
        initial_paths[i].push_back(n); // Return to depot
    }
    return initial_paths;
}

// TODO: Equalities are bad, maybe setting this to >=1 is better?
// TODO: Add in multi robot constraints (has to be visited N times).
inline void add_num_visits_constraint(IloEnv& env, IloModel& model, const std::vector<std::vector<std::vector<IloBoolVar>>>& x, int n, int m, const std::vector<int>& num_required_visits){
    for(int loc = 0; loc < n; loc++) {
        IloExpr expr(env);
        for(int agent = 0; agent < m; agent++) {
            IloExpr visits_per_agent(env);
            for(int from = 0; from < n + 1; from++) {
                if(from != loc) {
                    expr += x[agent][from][loc];
                    visits_per_agent += x[agent][from][loc];
                }
            }
            model.add(visits_per_agent <= 1); // Each agent can visit a location at most once.
            visits_per_agent.end();
        }

        // NOTE: This should be fine since there should be no shortcuts in the solution due to other constraints.
        // model.add(expr >= num_required_visits[loc]);
        model.add(expr == num_required_visits[loc]);

        // model.add(expr >= 1);
        // model.add(expr == 1);

        expr.end();
    }
}

inline void add_continuous_path_constraint(IloEnv& env, IloModel& model, const std::vector<std::vector<std::vector<IloBoolVar>>>& x, int n, int m){
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
}

inline void max_one_tour_per_agent_constraint(IloEnv& env, IloModel& model, const std::vector<std::vector<std::vector<IloBoolVar>>>& x, int n, int m){
    for(int agent = 0; agent < m; agent++) {
        IloExpr expr(env);
        for(int loc = 0; loc < n; loc++) {
            expr += x[agent][n][loc];
        }
        model.add(expr <= 1);
        expr.end();
    }
}

inline void must_return_to_depot(IloEnv& env, IloModel& model, const std::vector<std::vector<std::vector<IloBoolVar>>>& x, int n, int m){
    for(int agent = 0; agent < m; agent++) {
        IloExpr expr(env);
        for(int loc = 0; loc < n; loc++) {
            expr += x[agent][n][loc]; // Leaving depot
            expr -= x[agent][loc][n]; // Arriving at depot
        }
        model.add(expr == 0);
        expr.end();
    }
}

inline void subtour_elimination_constraint(IloEnv& env, IloModel& model, const std::vector<std::vector<IloNumVar>>& u, const std::vector<std::vector<std::vector<IloBoolVar>>>& x, int n, int m){
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
}

inline void print_paths(IloCplex& cplex, const std::vector<std::vector<std::vector<IloBoolVar>>>& x, int n, int m, const std::vector<std::vector<int>>& cost_matrix) {
    // Print out x variable:
    for(int agent = 0; agent < m; agent++) {
        cout << "Route for agent " << agent << ": ";
        for(int from = 0; from < n + 1; from++) {
            for(int to = 0; to < n + 1; to++) {
                if(from == to) continue;
                if(cplex.getValue(x[agent][from][to]) > 0.5) {
                    cout << from << "->" << to << "(c=" << cost_matrix[(from == n) ? (n + agent) : from][to] << ") ";
                }
            }
        }
        cout << endl;
    }
}

inline void set_MIP_start(IloNumVarArray& vars, IloNumArray& vals, const std::vector<std::vector<std::vector<IloBoolVar>>>& x, const std::vector<std::vector<int>>& cost_matrix, int n, int m, const std::vector<int>& num_required_visits) {
    std::vector<std::vector<int>> initial_paths = get_greedy_solution(cost_matrix, n, m, num_required_visits);

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
}


// NOTE: All tasks are also pivots, so num_tasks <= num_pivots.
inline int run_mtsp(int num_agents, int num_pivots, const std::vector<std::vector<int>>& cost_matrix, const std::vector<int>& current_costs, const std::vector<int>& num_required_visits) {
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
        add_num_visits_constraint(env, model, x, n, m, num_required_visits);

        // Constraint 2: Continuous path. Leaving loc - arriving at loc = 0 for all loc.
        add_continuous_path_constraint(env, model, x, n, m);

        // Constraint 3: Every agent has at most one tour. Done by ensuring every agent row sums to at most 1.
        max_one_tour_per_agent_constraint(env, model, x, n, m);

        // Constraint 4: Agents that leave their depot must return. Done by ensuring that sum of an agents row - sum of their column = 0.
        must_return_to_depot(env, model, x, n, m);

        // Constraint 5: Subtour elimination. Done using MTZ formulation.
        std::vector<std::vector<IloNumVar>> u(m, std::vector<IloNumVar>(n));
        for(int agent = 0; agent < m; agent++) {
            for(int city = 0; city < n; city++) {
                u[agent][city] = IloNumVar(env, 1, n, ILOFLOAT, ("u_" + to_string(agent) + "_" + to_string(city)).c_str());
            }
        }

        subtour_elimination_constraint(env, model, u, x, n, m);

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
        set_MIP_start(vars, vals, x, cost_matrix, n, m, num_required_visits);

        // Solve
        IloCplex cplex(model);
        cplex.setParam(IloCplex::Param::TimeLimit, 60); // 60 seconds time limit
        cplex.setOut(env.getNullStream());
        cplex.setParam(IloCplex::Param::Preprocessing::Presolve, 1); // enable presolve.
        cplex.setParam(IloCplex::Param::Emphasis::MIP, 1); // emphasize finding feasible solutions faster.
        cplex.setParam(IloCplex::CutsFactor, 1.0); // adjust cut aggressiveness.
        cplex.setParam(IloCplex::Param::Threads, 1); // Setting it to use 1 thread reduces overhead.

        cplex.addMIPStart(vars, vals, IloCplex::MIPStartAuto);

        auto end = std::chrono::high_resolution_clock::now();
        auto seconds_taken = std::chrono::duration<double>(end - start).count();
        METRICS.mtsp_setup_time += seconds_taken;
        METRICS.mtsp_total_calls += 1;

        start = std::chrono::high_resolution_clock::now();

        auto solve = cplex.solve();

        end = std::chrono::high_resolution_clock::now();
        seconds_taken = std::chrono::duration<double>(end - start).count();
        METRICS.mtsp_solver_runtime += seconds_taken;

        if(solve) {
            // printf("\n\nSolution found:\n");
            // printf("Agent costs: \n");
            // for(int agent = 0; agent < m; agent++) {
            //     int agent_cost = 0;
            //     for(int from = 0; from < n + 1; from++) {
            //         for(int to = 0; to < n + 1; to++) {
            //             if(from != to && to != n && cplex.getValue(x[agent][from][to]) > 0.5) {
            //                 agent_cost += cost_matrix[(from == n) ? (n + agent) : from][to];
            //             }
            //         }
            //     }
            //     printf("\tInitial cost: %d, total cost: %d\n", current_costs[agent], agent_cost + current_costs[agent]);
            // }
            // print_paths(cplex, x, n, m, cost_matrix);
            // if(num_required_visits.back() > 1 || num_required_visits[num_required_visits.size() - 2] > 1){
            //     printf("Note: Some locations required multiple visits.\n");
            //     exit(0);
            // }

            double result = cplex.getObjValue();
            int result_int = static_cast<int>(std::round(result));
            env.end();
            return result_int;

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
