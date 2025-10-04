#pragma once

#include <cstdlib>
#include <boost/dynamic_bitset.hpp>
#include "shared.hpp"

std::vector<std::vector<Position>> run_search(std::vector<Position> starts, const ScenarioConfig& scenario_config, const SolverConfig& solver_config);
