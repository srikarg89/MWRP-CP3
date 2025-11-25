import subprocess
import json
import time
import random

ASTAR_PD_TEMPLATE = {
    "heuristic": "TSP",
    "focal_method": "SOC",
    "cell_pruning_method": "CELL_THEN_PATH",
    "prune_pivots": False,
    "run_parallel": False,
    "expand_lowest_cost_agent_only": False,
    "max_pivots_generated": 8,
    "max_pivots_after_pruning": 10000000,
    "centralized_focal_epsilon": 1.0,
    "centralized_focal_heuristic_weight": 10000.0,
    "centralized_search_time_limit": 1.0,
    "centralized_astar_weight": 1.0,
    "run_decentralized_search": False,
    "decentralized_focal_epsilon": 1.0,
    "decentralized_focal_heuristic_weight": 1.5,
    "decentralized_search_time_limit": 10.0,
    "decentralized_astar_weight": 1.0,
    "max_decentralized_searches": 2
}

ASTAR_PD_PP_TEMPLATE = {
    "heuristic": "TSP",
    "focal_method": "SOC",
    "cell_pruning_method": "CELL_THEN_PATH",
    "prune_pivots": True,
    "run_parallel": False,
    "expand_lowest_cost_agent_only": False,
    "max_pivots_generated": 10000000,
    "max_pivots_after_pruning": 10000000,
    "centralized_focal_epsilon": 1.0,
    "centralized_focal_heuristic_weight": 10000.0,
    "centralized_search_time_limit": 1.0,
    "centralized_astar_weight": 1.0,
    "run_decentralized_search": False,
    "decentralized_focal_epsilon": 1.0,
    "decentralized_focal_heuristic_weight": 1.5,
    "decentralized_search_time_limit": 10.0,
    "decentralized_astar_weight": 1.0,
    "max_decentralized_searches": 2
}

ASTAR_PD_PC_TEMPLATE = {
    "heuristic": "TSP",
    "focal_method": "SOC",
    "cell_pruning_method": "CELL_THEN_PATH",
    "prune_pivots": False,
    "run_parallel": True,
    "expand_lowest_cost_agent_only": False,
    "max_pivots_generated": 8,
    "max_pivots_after_pruning": 10000000,
    "centralized_focal_epsilon": 1.0,
    "centralized_focal_heuristic_weight": 10000.0,
    "centralized_search_time_limit": 1.0,
    "centralized_astar_weight": 1.0,
    "run_decentralized_search": False,
    "decentralized_focal_epsilon": 1.0,
    "decentralized_focal_heuristic_weight": 1.5,
    "decentralized_search_time_limit": 10.0,
    "decentralized_astar_weight": 1.0,
    "max_decentralized_searches": 2
}

FINAL_MWRP_TEMPLATE = {
    "heuristic": "TSP",
    "focal_method": "SOC",
    "cell_pruning_method": "CELL_THEN_PATH",
    "prune_pivots": True,
    "run_parallel": True,
    "expand_lowest_cost_agent_only": False,
    "max_pivots_generated": 10000000,
    "max_pivots_after_pruning": 10000000,
    "centralized_focal_epsilon": 1.0,
    "centralized_focal_heuristic_weight": 10000.0,
    "centralized_search_time_limit": 1.0,
    "centralized_astar_weight": 1.0,
    "run_decentralized_search": False,
    "decentralized_focal_epsilon": 1.0,
    "decentralized_focal_heuristic_weight": 1.5,
    "decentralized_search_time_limit": 10.0,
    "decentralized_astar_weight": 1.0,
    "max_decentralized_searches": 2
}

# method_names = ["CR", "CR_PP", "CR_PC", "FINAL_MWRP"]
# methods = [ASTAR_PD_TEMPLATE, ASTAR_PD_PP_TEMPLATE, ASTAR_PD_PC_TEMPLATE, FINAL_MWRP_TEMPLATE]
method_names = ["CR_PP", "CR_PC", "FINAL_MWRP"]
methods = [ASTAR_PD_PP_TEMPLATE, ASTAR_PD_PC_TEMPLATE, FINAL_MWRP_TEMPLATE]
method_names = method_names[::-1]
methods = methods[::-1]

# method_names = ["CR_PP", "CR_PC", "FINAL_MWRP"]
# methods = [ASTAR_PD_PP_TEMPLATE, ASTAR_PD_PC_TEMPLATE, FINAL_MWRP_TEMPLATE]

# method_names = ["CR"]
# methods = [ASTAR_PD_TEMPLATE]

num_experiments = 1


# method_names = ["OG_MWRP"]
# methods = [OG_MWRP_TEMPLATE]
# num_experiments = 1


# MAP_NAME = "../maps/custom-19-19.map"
# SCEN_CONFIG = "../configs/19_by_19_maze_experiment.json"

# MAP_NAME = "../maps/custom-13-13.map"
# SCEN_CONFIG = "../configs/13_by_13_maze_experiment.json"

# MAP_NAME = "../maps/den020d.map"
# MAP_NAME = "../maps/den101d.map"
# MAP_NAMES = ["../maps/maze-32-32-2.map", "../maps/den101d.map", "../maps/den020d.map"]
# PREMOVED_AGENT_LOCS = [[[1, 1], [1, 31], [31, 31]], [[41, 2], [12, 40]], [[22, 2]]]
# NUM_AGENT_LOCS = [3, 2, 1]

# MAP_NAMES = ["../maps/huge/AR0308SR.map"]
# PREMOVED_AGENT_LOCS = [[[44, 108]]]
# NUM_AGENT_LOCS = [1]

# MAP_NAMES = ["../maps/maze-32-32-2.map", "../maps/maze-32-32-2.map", "../maps/maze-32-32-2.map", "../maps/maze-32-32-2.map", "../maps/maze-32-32-2.map"]
# PREMOVED_AGENT_LOCS = [[[1, 1]], [[1, 1], [1, 31]], [[1, 1], [1, 31], [31, 31]], [[1, 1], [1, 31], [31, 31], [31, 1]], [[1, 1], [1, 31], [31, 31], [31, 1], [16,13]]]
MAP_NAMES = ["../maps/den101d.map", "../maps/den101d.map", "../maps/den101d.map", "../maps/den101d.map", "../maps/den101d.map"]
PREMOVED_AGENT_LOCS = [[[41, 2]], [[41, 2], [12,40]], [[41, 2], [12,40], [46, 38]]]
NUM_AGENT_LOCS = [1, 2, 3]


SCEN_CONFIG = "../configs/opt_better_experiment.json"

def get_random_agent_starts(map_name, num_agents):
    with open(map_name) as f:
        lines = f.readlines()
        map_lines = lines[4:] # Skip the first four header lines
        map_lines = [line.strip() for line in map_lines if line.strip()]  # Remove any empty lines
        map = [[0 if c == '.' else 1 for c in line.strip()] for line in map_lines]
    
    free_cells = []
    for y in range(len(map)):
        for x in range(len(map[0])):
            if map[y][x] == 0:
                free_cells.append([x, y])

    return random.sample(free_cells, num_agents)


# for RANDOM, RANDOM_INSTANCES in [(False, 1), (True, 20)]:
for RANDOM, RANDOM_INSTANCES in [(False, 1)]:
    for i in range(len(MAP_NAMES)):
        map_name, num_agent_starts = MAP_NAMES[i], NUM_AGENT_LOCS[i]
        num_experiments = RANDOM_INSTANCES if RANDOM else 1
        for experiment_id in range(num_experiments):
            if RANDOM:
                agent_locs = get_random_agent_starts(map_name, num_agent_starts)
            else:
                agent_locs = PREMOVED_AGENT_LOCS[i]

            scenario_config = {
                "agents": agent_locs,
                "tasks": [],
                "map": map_name.removeprefix("../maps/"),
                "movement": "FOUR_WAY_MOVEMENT",
                "los": "BRES_LOS"
            }
            with open(SCEN_CONFIG, "w") as f:
                json.dump(scenario_config, f, indent=4)

            for method_name, method in zip(method_names, methods):
                input_config = method.copy()
                with open("../solver.json", "w") as f:
                    json.dump(input_config, f, indent=4)

                print(f"Running experiment with {num_agent_starts} agent starts, experiment id {experiment_id}, method {method_name}")
                start_time = time.time()
                # result = subprocess.run(["./run", SCEN_CONFIG, "../solver.json"], cwd="/home/srikar/Documents/Research/SearchAndTAPFWithPTC/build", check=True, capture_output=True, text=True)
                result = subprocess.run(["./run", SCEN_CONFIG, "../solver.json"], cwd="/home/srikar/Documents/Research/SearchAndTAPFWithPTC/build", check=True)
                measured_time = time.time() - start_time

                search_time = -1.0
                total_time = -1.0

                # lines = result.stdout.strip().split("\n")
                # for line in lines:
                #     line = line.strip()
                #     if line.startswith("Total search time taken:"):
                #         search_time = float(line.split()[-2].strip())
                #     elif line.startswith("Total time taken:"):
                #         total_time = float(line.split()[-2].strip())
                
                measured_time_ms = int(measured_time * 1000)
                search_time_ms = int(search_time * 1000)
                total_time_ms = int(total_time * 1000)

                print("\tMethod", method_name, " took", measured_time_ms, "ms to run")
                if RANDOM:
                    results = open("opt_better_experiment_results_random.csv", "a+")
                else:
                    results = open("opt_better_experiment_results_nonrandom2.csv", "a+")
                results.write(f"{map_name},{method_name},{num_agent_starts},{experiment_id},{search_time_ms},{total_time_ms},{measured_time_ms}\n")
                results.close()
