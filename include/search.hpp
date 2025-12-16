#pragma once

#include <cstdlib>
#include <boost/dynamic_bitset.hpp>
#include "shared.hpp"
#include "file_utils.hpp"

std::vector<std::vector<Position>> run_search(int start_timestep, std::vector<Position> starts, boost::dynamic_bitset<> start_seen, const Map& map, const SolverConfig& solver_config, const Lookup& lookup);
