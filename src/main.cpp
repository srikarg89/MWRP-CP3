#include <iostream>
#include <chrono>
#include "watchman.hpp"

#define new new_
#define class class_

extern "C"
{
#include "concorde.h"
}

#undef new
#undef class

using namespace std;

// extern "C" { #include <concorde.h> }
// function that calls Concorde solver functions
std::vector<int> solving_tsp_concorde(const vector<vector<int>>& dist){
  // TODO: TSP For less than 4 cities.
  // creating a sequential tour
  if (dist.size() > 4) { // TSP for more than 4 cities
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

    CCrandstate rstate;
    CCutil_sprand(rand(), &rstate);

    int *out_tour = CC_SAFE_MALLOC(ncount, int);
    char *name = CCtsp_problabel(" ");

    // Solve TSP. The commented out code is another method that can be called, but it seems to be about 30% slower.
    // CCdatagroup dat;
    // CCutil_init_datagroup(&dat);
    // CCutil_graph2dat_matrix(ncount, ecount, elist, elen, 1, &dat);
    // CCtsp_solve_dat(ncount, &dat, /* in_tour = */ NULL, out_tour, /* in_val = */ NULL, &optval, &success, &foundtour, name, NULL, &hit_timebound, /* silent = */ 1, &rstate);
    CCtsp_solve_sparse(ncount, ecount, elist, elen, /* in_tour = */ NULL, out_tour, /* in_val = */ NULL, &optval, &success, &foundtour, name, NULL, &hit_timebound, /* silent = */ 1, &rstate);

    // Check return code.
    if(success != 1 || foundtour != 1) {
      fprintf(stderr, "TSP solver failed to find a solution: %d, %d\n", success, foundtour);
      exit(1);
    }

    // Copy in solution.
    std::vector<int> tour;
    for (int i = 0; i < ncount; i++) {
      tour.push_back(out_tour[i]);
    }
    printf("Optimal cost: %f\n", optval);

    // Free allocated variables.
    CC_IFFREE(elist, int);
    CC_IFFREE(elen, int);
    CC_IFFREE(out_tour, int);
    CC_IFFREE(name, char);

    return tour;
  }
}
int main()
{
  // distance matrix and vector initialization
  std::vector<std::vector<int>> distance = {
      {0, 29, 20, 21, 16, 31, 100, 12, 4, 31, 18, 23, 14, 12, 24, 45, 67, 34, 89, 54, 32, 25, 17, 48, 62},
      {29, 0, 15, 29, 28, 40, 72, 21, 29, 41, 12, 28, 21, 29, 40, 50, 60, 33, 75, 58, 22, 27, 31, 45, 55},
      {20, 15, 0, 15, 14, 25, 81, 9, 23, 27, 13, 16, 17, 20, 36, 42, 57, 28, 66, 48, 21, 26, 19, 38, 50},
      {21, 29, 15, 0, 12, 17, 82, 21, 27, 23, 11, 15, 19, 25, 31, 38, 46, 29, 70, 52, 24, 30, 18, 36, 47},
      {16, 28, 14, 12, 0, 20, 95, 15, 18, 20, 17, 22, 16, 19, 27, 33, 44, 26, 71, 50, 20, 22, 21, 34, 46},
      {31, 40, 25, 17, 20, 0, 87, 28, 31, 25, 14, 19, 24, 28, 34, 40, 53, 32, 65, 55, 26, 30, 24, 39, 49},
      {100, 72, 81, 82, 95, 87, 0, 90, 94, 99, 83, 77, 85, 88, 92, 105, 97, 89, 60, 73, 84, 91, 93, 102, 108},
      {12, 21, 9, 21, 15, 28, 90, 0, 15, 19, 12, 14, 13, 16, 23, 31, 40, 20, 59, 41, 18, 22, 16, 29, 37},
      {4, 29, 23, 27, 18, 31, 94, 15, 0, 32, 19, 23, 12, 11, 21, 28, 39, 25, 63, 47, 16, 20, 14, 33, 44},
      {31, 41, 27, 23, 20, 25, 99, 19, 32, 0, 24, 29, 21, 24, 30, 37, 50, 31, 68, 53, 23, 28, 27, 40, 52},
      {18, 12, 13, 11, 17, 14, 83, 12, 19, 24, 0, 13, 15, 17, 26, 33, 41, 25, 62, 44, 19, 21, 15, 32, 42},
      {23, 28, 16, 15, 22, 19, 77, 14, 23, 29, 13, 0, 18, 21, 28, 36, 48, 27, 67, 46, 20, 25, 22, 37, 49},
      {14, 21, 17, 19, 16, 24, 85, 13, 12, 21, 15, 18, 0, 9, 20, 27, 36, 23, 58, 39, 17, 19, 16, 30, 40},
      {12, 29, 20, 25, 19, 28, 88, 16, 11, 24, 17, 21, 9, 0, 18, 25, 34, 22, 55, 38, 15, 18, 13, 28, 36},
      {24, 40, 36, 31, 27, 34, 92, 23, 21, 30, 26, 28, 20, 18, 0, 15, 29, 18, 48, 31, 20, 23, 25, 27, 35},
      {45, 50, 42, 38, 33, 40, 105, 31, 28, 37, 33, 36, 27, 25, 15, 0, 22, 12, 40, 25, 24, 29, 31, 22, 30},
      {67, 60, 57, 46, 44, 53, 97, 40, 39, 50, 41, 48, 36, 34, 29, 22, 0, 14, 29, 18, 30, 35, 39, 28, 33},
      {34, 33, 28, 29, 26, 32, 89, 20, 25, 31, 25, 27, 23, 22, 18, 12, 14, 0, 36, 20, 19, 24, 26, 19, 27},
      {89, 75, 66, 70, 71, 65, 60, 59, 63, 68, 62, 67, 58, 55, 48, 40, 29, 36, 0, 22, 49, 54, 61, 42, 31},
      {54, 58, 48, 52, 50, 55, 73, 41, 47, 53, 44, 46, 39, 38, 31, 25, 18, 20, 22, 0, 27, 32, 35, 24, 29},
      {32, 22, 21, 24, 20, 26, 84, 18, 16, 23, 19, 20, 17, 15, 20, 24, 30, 19, 49, 27, 0, 14, 16, 21, 28},
      {25, 27, 26, 30, 22, 30, 91, 22, 20, 28, 21, 25, 19, 18, 23, 29, 35, 24, 54, 32, 14, 0, 18, 26, 33},
      {17, 31, 19, 18, 21, 24, 93, 16, 14, 27, 15, 22, 16, 13, 25, 31, 39, 26, 61, 35, 16, 18, 0, 28, 36},
      {48, 45, 38, 36, 34, 39, 102, 29, 33, 40, 32, 37, 30, 28, 27, 22, 28, 19, 42, 24, 21, 26, 28, 0, 19},
      {62, 55, 50, 47, 46, 49, 108, 37, 44, 52, 42, 49, 40, 36, 35, 30, 33, 27, 31, 29, 28, 33, 36, 19, 0}
  };



  // vector<vector<int>> distance = {{0, 2, 9, 10, 7},
  //                                 {1, 0, 6, 4, 3},
  //                                 {15, 7, 0, 8, 12},
  //                                 {6, 3, 12, 0, 5},
  //                                 {10, 4, 8, 3, 0}};

  // vector<vector<int>> distance = {{0, 2, 9},
  //                                 {1, 0, 6},
  //                                 {15, 7, 0}};

  vector<int> tour = solving_tsp_concorde(distance);
  // prints the solution
  for (int i = 0; i < tour.size(); i++)
  {
    cout << tour[i] << " ";
  }
  // build here the solution to the original problem
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
