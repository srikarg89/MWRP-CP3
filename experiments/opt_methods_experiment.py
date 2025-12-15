import subprocess
import json
import time
import random

OG_MWRP_TEMPLATE = {
    "heuristic": "TSP",
    "focal_method": "SOC",
    "cell_pruning_method": "NONE",
    "prune_pivots": False,
    "run_parallel": False,
    "max_pivots_generated": 8,
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

REDUCTION_MWRP_TEMPLATE = {
    "heuristic": "TSP",
    "focal_method": "SOC",
    "cell_pruning_method": "PATH",
    "prune_pivots": False,
    "run_parallel": False,
    "max_pivots_generated": 8,
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
    "cell_pruning_method": "PATH",
    "prune_pivots": True,
    "run_parallel": True,
    "max_pivots_generated": 10000000,
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

method_names = ["OG_MWRP", "REDUCTION_MWRP", "FINAL_MWRP"]
methods = [OG_MWRP_TEMPLATE, REDUCTION_MWRP_TEMPLATE, FINAL_MWRP_TEMPLATE]
num_experiments = 1

method_names = method_names[::-1]
methods = methods[::-1]


# method_names = ["OG_MWRP"]
# methods = [OG_MWRP_TEMPLATE]
# num_experiments = 1


# MAP_NAME = "../maps/custom-19-19.map"
# SCEN_CONFIG = "../configs/opt_better_experiment.json"

# MAP_NAME = "../maps/custom-13-13.map"
# SCEN_CONFIG = "../configs/13_by_13_maze_experiment.json"
# AGENT_LOCS = [[0, 0], [12, 12], [1, 12], [12, 1], [12, 4]]
# AGENT_LOCS = [[0, 0], [0, 0], [0, 0], [0, 0], [0, 0]]
# NUM_AGENT_LOCS = [1, 2, 3, 4, 5]

# MAP_NAME = "../maps/custom-11-11.map"
# SCEN_CONFIG = "../configs/11_by_11_maze_experiment.json"
# AGENT_LOCS = [[0, 0], [0, 0], [0, 0], [0, 0], [0, 0]]
# AGENT_LOCS = [[0, 0], [0, 0], [0, 0], [0, 0], [0, 0]]
# NUM_AGENT_LOCS = [1, 2, 3, 4, 5]

# MAP_NAMES = ["../maps/custom-11-11.map", "../maps/custom-13-13.map", "../maps/custom-19-19.map", "../maps/maze-32-32-2.map"]
# SCEN_CONFIG = "../configs/test.json"
# AGENT_LOCS = [[[0, 0]], [[0, 0]], [[0, 0]], [[1, 1]]]
# NUM_AGENT_LOCS = [1, 1, 1, 1]


MAP_NAMES = ["../maps/custom-19-19.map"]
SCEN_CONFIG = "../configs/test.json"
AGENT_LOCS = [[[18, 18]]]
NUM_AGENT_LOCS = [1]


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


for i in range(len(NUM_AGENT_LOCS)):
    for experiment_id in range(num_experiments):
        map_name, num_agent_starts = MAP_NAMES[i], NUM_AGENT_LOCS[i]
        # agent_locs = get_random_agent_starts(MAP_NAME, num_agent_starts)
        agent_locs = AGENT_LOCS[i]

        scenario_config = {
            "agents": agent_locs,
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
            time_taken = time.time() - start_time
            time_ms = int(time_taken * 1000)

            # num_expanded = -1
            # lines = result.stdout.strip().split("\n")
            # for line in lines:
            #     line = line.strip()
            #     if line.startswith("Total nodes expanded:"):
            #         num_expanded = int(line.split()[-1].strip())
            #         break
            
            # print("\tMethod", method_name, "expanded", num_expanded, " nodes and took", time_ms, "ms to run")
            print("\tMethod", method_name, " nodes and took", time_ms, "ms to run")
            results = open("opt_experiment_results_test_2.csv", "a+")
            results.write(f"{map_name},{method_name},{num_agent_starts},{experiment_id},{time_ms}\n")
            # results.write(f"{MAP_NAME},{method_name},{num_agent_starts},{experiment_id},{num_expanded},{time_ms}\n")
            results.close()


