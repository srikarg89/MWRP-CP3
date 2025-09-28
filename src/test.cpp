#include <iostream>
#include "mtsp.hpp"

int main(int argc, char** argv) {
    int m = 2;
    int n = 5;

    vector<vector<int>> cost_matrix = {
        {0,2,9,10,7},
        {2,0,6,4,3},
        {9,6,0,8,5},
        {10,4,8,0,6},
        {7,3,5,6,0},
        {0,0,0,0,0},
        {0,0,0,0,0},
    };

    int makespan = run_mtsp(m, n, cost_matrix);
    std::cout << "Makespan: " << makespan << std::endl;

    return 0;
}
