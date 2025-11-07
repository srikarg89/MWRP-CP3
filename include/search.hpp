#pragma once

#include <cstdlib>
#include <boost/dynamic_bitset.hpp>
#include "shared.hpp"
#include "file_utils.hpp"

std::vector<std::vector<Position>> run_search(int start_timestep, std::vector<Position> starts, std::vector<Task> incomplete_tasks, boost::dynamic_bitset<> start_seen, const Map& map, const SolverConfig& solver_config, const Lookup& lookup, const std::unordered_map<std::string, std::vector<PastSolution>>& solution_history);
