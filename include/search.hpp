#pragma once

#include <cstdlib>
#include <boost/dynamic_bitset.hpp>
#include "shared.hpp"

std::vector<std::vector<Position>> run_search(std::vector<Position> starts, std::vector<Position> incomplete_tasks_pos, const Map& map, const SolverConfig& solver_config);
