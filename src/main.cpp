#include <iostream>
#include "watchman.hpp"

#define new new_
#define class class_

extern "C" {
  #include "concorde.h"
}

#undef new
#undef class

using namespace std;

// extern "C" { #include <concorde.h> }
//function that calls Concorde solver functions
void solving_tsp_concorde(vector<vector<int>>* dist, vector<int>* tour){
//creating a sequential tour
for(int i = 0; i < tour->size(); i++){
tour->at(i) = i;
}
if(dist->size() > 4 ){//TSP for more than 4 cities
int rval = 0;
int seed = rand();
double szeit, optval, *in_val, *timebound;
int ncount, success, foundtour, hit_timebound = 0;
int *in_tour = (int *) NULL;
int *out_tour = (int *) NULL;
CCrandstate rstate;
char *name = (char *) NULL;
static int silent = 1;
CCutil_sprand(seed, &rstate);
in_val = (double *) NULL;
timebound = (double *) NULL;
ncount = dist->size();
int ecount = (ncount * (ncount - 1)) / 2;
int *elist = new int[ecount * 2];
int *elen = new int[ecount];
int edge = 0;
int edgeWeight = 0;
for (int i = 0; i < ncount; i++) {
for (int j = i + 1; j < ncount; j++) {
if (i != j) {
elist[edge] = i;
elist[edge + 1] = j;
elen[edgeWeight] = dist->at(i)[j];
edgeWeight++;
edge = edge + 2;
}
}
}
out_tour = CC_SAFE_MALLOC (ncount, int);
name = CCtsp_problabel(" ");
CCdatagroup dat;
CCutil_init_datagroup (&dat);
rval = CCutil_graph2dat_matrix (ncount, ecount, elist, elen, 1, &dat);
rval = CCtsp_solve_dat (ncount, &dat, in_tour,  out_tour, NULL, &optval, &success, &foundtour, name, timebound, &hit_timebound, silent, &rstate);
for (int i = 0; i < ncount; i++) {
tour->at(i) = out_tour[i];
}
szeit = CCutil_zeit();
CC_IFFREE (elist, int);
CC_IFFREE (elen, int);
CC_IFFREE (out_tour, int);
// CC_IFFREE (probname, char);
}
}
int main(){
//distance matrix and vector initialization
vector<vector<int>> distance = { {0, 2, 9, 10, 7},
{1, 0, 6, 4, 3},
{15, 7, 0, 8, 12},
{6, 3, 12, 0, 5},
{10, 4, 8, 3, 0} };
vector<int> tour(5);
solving_tsp_concorde(&distance, &tour);
//prints the solution
for(int i = 0; i < tour.size(); i++){
cout<<tour[i]<<" ";
}
//build here the solution to the original problem
return 0;
}

// int main(int argc, char** argv) {
//     // --- Problem setup ---
//     const int n = 5;
//     double coords[n][2] = {
//         {0.0, 0.0},
//         {1.0, 0.0},
//         {1.0, 1.0},
//         {0.0, 1.0},
//         {0.5, 0.5}
//     };

//     // Initialize Concorde data structure
//     CCdatagroup dat;
//     CCutil_init_datagroup(&dat);
//     // dat.ncount = n;
//     dat.x = new double[n];
//     dat.y = new double[n];
//     for (int i = 0; i < n; i++) {
//         dat.x[i] = coords[i][0];
//         dat.y[i] = coords[i][1];
//     }
//     dat.norm = CC_EUCLIDEAN;

//     // --- Solver parameters ---
//     int* in_tour = nullptr;       // optional initial tour
//     int out_tour[n];              // optimal tour output
//     double* in_val = nullptr;     // optional initial cost
//     double optval;                // optimal tour length
//     int success;                  // 1 = solved successfully
//     int foundtour;                // 1 = tour returned
//     char* name = nullptr;         // problem name (optional)
//     double* timebound = nullptr;  // no time limit
//     int hit_timebound;            // 1 if stopped due to time limit
//     int silent = 1;               // suppress output
//     CCrandstate* rstate = nullptr;// default random state

//     // --- Solve TSP ---
//     int rval = CCtsp_solve_dat(
//         n, &dat,
//         in_tour,
//         out_tour,
//         in_val,
//         &optval,
//         &success,
//         &foundtour,
//         name,
//         timebound,
//         &hit_timebound,
//         silent,
//         rstate
//     );

//     if (rval) {
//         std::cerr << "Concorde solver failed with code " << rval << "\n";
//         delete[] dat.x;
//         delete[] dat.y;
//         return 1;
//     }

//     if (success && foundtour) {
//         std::cout << "Optimal tour length: " << optval << "\n";
//         std::cout << "Tour order: ";
//         for (int i = 0; i < n; i++) std::cout << out_tour[i] << " ";
//         std::cout << "\n";
//     } else {
//         std::cout << "Solver finished but no tour returned.\n";
//     }

//     // --- Cleanup ---
//     delete[] dat.x;
//     delete[] dat.y;

//     return 0;


//     // int n = 5;
//     // double coords[5][2] = {
//     //     {0, 0}, {1, 0}, {1, 1}, {0, 1}, {0.5, 0.5}
//     // };

//     // CCdatagroup dat;  
//     // CCutil_init_datagroup(&dat);

//     // // Tell Concorde we’re using 2D Euclidean distances
//     // dat.x = new double[n];
//     // dat.y = new double[n];
//     // for (int i = 0; i < n; i++) {
//     //     dat.x[i] = coords[i][0];
//     //     dat.y[i] = coords[i][1];
//     // }
//     // dat.norm = CC_EUCLIDEAN;

//     // int *tour = new int[n];
//     // double val = 0.0;
//     // int success;

//     // // Solve TSP
//     // int rval = CCtsp_solve_dat(n, &dat, NULL, tour, NULL, &val, &success, NULL, NULL, NULL, NULL, 0, NULL);
//     // if (rval) {
//     //     std::cerr << "Concorde failed with code " << rval << std::endl;
//     //     return 1;
//     // }

//     // std::cout << "Optimal tour length: " << val << std::endl;
//     // std::cout << "Tour order: ";
//     // for (int i = 0; i < n; i++) {
//     //     std::cout << tour[i] << " ";
//     // }
//     // std::cout << std::endl;

//     // delete[] tour;
//     // delete[] dat.x;
//     // delete[] dat.y;
//     // return 0;

//     // // Setup scenario config.
//     // ScenarioConfig scenario_config = ScenarioConfig::from_json(argv[1]);
//     // MovementType movement_type = MovementType::FOUR_WAY_MOVEMENT;

//     // // Setup solver config.
//     // std::string heuristic_str = argv[2];
//     // HeuristicType heuristic_type;
//     // if(heuristic_str == "BFS") {
//     //     heuristic_type = HeuristicType::BFS;
//     // } else if(heuristic_str == "SINGLETON") {
//     //     heuristic_type = HeuristicType::SINGLETON;
//     // } else if(heuristic_str == "MST") {
//     //     heuristic_type = HeuristicType::MST;
//     // } else if(heuristic_str == "TSP") {
//     //     heuristic_type = HeuristicType::TSP;
//     // } else {
//     //     throw std::runtime_error("Invalid heuristic type: " + heuristic_str);
//     // }

//     // std::vector<Position> solution = watchman::run_watchman(scenario_config.agent_starts[0], scenario_config.los_type, scenario_config.map, scenario_config.movement_type, heuristic_type);
//     // printf("Solution size: %ld\n", solution.size());
//     // for(Position pos : solution){
//     //     printf("\t%s\n", pos.toString().c_str());
//     // }

//     return 0;
// }
