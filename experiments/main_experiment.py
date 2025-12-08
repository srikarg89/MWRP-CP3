import subprocess
import json
import time
import random

import time
import os

import psutil

def check_pid(pid):

    if int(pid) in psutil.pids(): ## Check list of PIDs
        return True ## Running
    else:
        return False ## Not Running

# while check_pid(5316):
#     print("Waiting for previous run to finish...")
#     time.sleep(300)




W_VALUE = 1.5
TIME_LIMIT = 300.0
PARALLEL_BATCH_SIZE = 100

MWRCP3_TEMPLATE = {
    "heuristic": "LAZY",
    "focal_method": "SOC",
    "cell_pruning_method": "CELL_THEN_PATH",
    "prune_pivots": True,
    "run_parallel": True,
    "expand_lowest_cost_agent_only": False,
    "max_pivots_generated": 10000000,
    "max_pivots_after_pruning": 10000000,
    "parallel_batch_size": PARALLEL_BATCH_SIZE,
    "centralized_focal_epsilon": 1.0,
    "centralized_focal_heuristic_weight": 10000.0,
    "centralized_search_time_limit": 0.0001,
    "centralized_hard_search_time_limit": TIME_LIMIT,
    "centralized_astar_weight": 1.0,
    "run_decentralized_search": False,
    "decentralized_focal_epsilon": 1.0,
    "decentralized_focal_heuristic_weight": 1.5,
    "decentralized_search_time_limit": 10.0,
    "decentralized_hard_search_time_limit": 500.0,
    "decentralized_astar_weight": 1.0,
    "max_decentralized_searches": 2
}

OG_MWRP_TEMPLATE = MWRCP3_TEMPLATE.copy()
OG_MWRP_TEMPLATE["cell_pruning_method"] = "NONE"
OG_MWRP_TEMPLATE["prune_pivots"] = False
OG_MWRP_TEMPLATE["run_parallel"] = False
OG_MWRP_TEMPLATE["max_pivots_generated"] = 8
OG_MWRP_TEMPLATE["parallel_batch_size"] = 1

MWRP_CPD_TEMPLATE = MWRCP3_TEMPLATE.copy()
MWRP_CPD_TEMPLATE["prune_pivots"] = False
MWRP_CPD_TEMPLATE["run_parallel"] = False
MWRP_CPD_TEMPLATE["max_pivots_generated"] = 8
MWRP_CPD_TEMPLATE["parallel_batch_size"] = 1

MxWAstar_TEMPLATE = MWRCP3_TEMPLATE.copy()
MxWAstar_TEMPLATE["centralized_astar_weight"] = W_VALUE

FOCAL_SOC_TEMPLATE = MWRCP3_TEMPLATE.copy()
FOCAL_SOC_TEMPLATE["focal_method"] = "SOC"
FOCAL_SOC_TEMPLATE["centralized_focal_epsilon"] = W_VALUE

FOCAL_MOC_TEMPLATE = MWRCP3_TEMPLATE.copy()
FOCAL_MOC_TEMPLATE["focal_method"] = "MOC"
FOCAL_MOC_TEMPLATE["centralized_focal_epsilon"] = W_VALUE

MxWAsta2_TEMPLATE = MxWAstar_TEMPLATE.copy()
MxWAsta2_TEMPLATE["centralized_astar_weight"] = 2.0

FOCAL_SOC2_TEMPLATE = FOCAL_SOC_TEMPLATE.copy()
FOCAL_SOC2_TEMPLATE["centralized_focal_epsilon"] = 2.0

FOCAL_MOC2_TEMPLATE = FOCAL_MOC_TEMPLATE.copy()
FOCAL_MOC2_TEMPLATE["centralized_focal_epsilon"] = 2.0



MWRCP3_TSP_TEMPLATE = MWRCP3_TEMPLATE.copy()
MWRCP3_TSP_TEMPLATE["heuristic"] = "TSP"

MWRP_CPD_PP_TEMPLATE = MWRP_CPD_TEMPLATE.copy()
MWRP_CPD_PP_TEMPLATE["prune_pivots"] = True
MWRP_CPD_PP_TEMPLATE["max_pivots_generated"] = 10000000

MWRP_CPD_PHC_TEMPLATE = MWRP_CPD_TEMPLATE.copy()
MWRP_CPD_PHC_TEMPLATE["run_parallel"] = True
MWRP_CPD_PHC_TEMPLATE["parallel_batch_size"] = PARALLEL_BATCH_SIZE


# MWRCP3_TEMPLATE["centralized_hard_search_time_limit"] = 100.0

num_experiments = 10


####### EXPERIMENT CONFIGURATION ########
method_names = ["MWRP_CP3", "MxWAstar", "FOCAL_SOC", "FOCAL_MOC", "MWRP_CPD", "OG_MWRP"]
methods = [MWRCP3_TEMPLATE, MxWAstar_TEMPLATE, FOCAL_SOC_TEMPLATE, FOCAL_MOC_TEMPLATE, MWRP_CPD_TEMPLATE, OG_MWRP_TEMPLATE]

method_names = ["MWRP_CP3", "MxWAstar", "MxWAstar2", "FOCAL_SOC", "FOCAL_SOC2", "FOCAL_MOC", "FOCAL_MOC2", "MWRP_CPD", "OG_MWRP"]
methods = [MWRCP3_TEMPLATE, MxWAstar_TEMPLATE, MxWAsta2_TEMPLATE, FOCAL_SOC_TEMPLATE, FOCAL_SOC2_TEMPLATE, FOCAL_MOC_TEMPLATE, FOCAL_MOC2_TEMPLATE, MWRP_CPD_TEMPLATE, OG_MWRP_TEMPLATE]

# # Mazes Map Scaling
# MAP_NAMES = ["../maps/custom-11-11.map", "../maps/custom-13-13.map", "../maps/custom-19-19.map", "../maps/maze-32-32-2.map"]
# SCEN_CONFIG = "../configs/test.json"
# NUM_AGENT_LOCS = [2, 2, 2, 2]

# # Mazes Agent Scaling
# MAP_NAMES = ["../maps/custom-13-13.map"] * 6
# SCEN_CONFIG = "../configs/test.json"
# NUM_AGENT_LOCS = [1, 2, 3, 4, 5, 6]

# Games Map Scaling
# MAP_NAMES = ["../maps/den202d.map", "../maps/den207d.map", "../maps/den101d.map"]
# MAP_NAMES = ["../maps/den207d.map", "../maps/den101d.map"]
MAP_NAMES = ["../maps/maps2/lak105d.map", "../maps/maps2/lak104d.map", "../maps/maps2/lak103d.map"]
MAP_NAMES = ["../maps/maps2/lak105d.map"]
SCEN_CONFIG = "../configs/test.json"
NUM_AGENT_LOCS = [2] * len(MAP_NAMES)


# MAP_NAMES = MAP_NAMES[3:]
# NUM_AGENT_LOCS = NUM_AGENT_LOCS[3:]


# # ###### ABLATION EXPERIMENT ######
# method_names = ["MWRP_CP3", "MWRP_CPD_PP", "MWRP_CPD_PHC"]
# methods = [MWRCP3_TEMPLATE, MWRP_CPD_PP_TEMPLATE, MWRP_CPD_PHC_TEMPLATE]

# # MAP_NAMES = ["../maps/maze-32-32-2.map", "../maps/den101d.map", "../maps/maze-32-32-2.map", "../maps/custom-19-19.map"]
# # MAP_NAMES = ["../maps/huge/AR0401SR.map"]
# # MAP_NAMES = ["../maps/den207d.map", "../maps/huge/AR0607SR.map", "../maps/huge/orz101d.map"]
# # MAP_NAMES = ["../maps/huge/AR0607SR.map"] * 3
# # MAP_NAMES = ["../maps/den207d.map"] * 3
# # MAP_NAMES = ["../maps/den202d.map"] * 3
# # MAP_NAMES = ["../maps2/den009d.map"] * 3
# # MAP_NAMES += ["../maps2/den998d.map"] * 3
# # MAP_NAMES += ["../maps2/hrt002d.map"] * 3
# # MAP_NAMES += ["../maps2/lak103d.map"] * 3
# # MAP_NAMES += ["../maps2/lak104d.map"] * 3
# # MAP_NAMES += ["../maps2/lak105d.map"] * 3
# # MAP_NAMES += ["../maps2/lak107d.map"] * 3
# # MAP_NAMES += ["../maps2/lak108d.map"] * 3
# # MAP_NAMES += ["../maps2/lak109d.map"] * 3

# # MAP_NAMES = ["../maps/maze-32-32-2.map"] * 3
# MAP_NAMES = ["../maps/den101d.map"] * 3

# SCEN_CONFIG = "../configs/test.json"
# NUM_AGENT_LOCS = [2]


# def get_random_agent_starts(map_name, num_agents):
#     with open(map_name) as f:
#         lines = f.readlines()
#         map_lines = lines[4:] # Skip the first four header lines
#         map_lines = [line.strip() for line in map_lines if line.strip()]  # Remove any empty lines
#         map = [[0 if c == '.' else 1 for c in line.strip()] for line in map_lines]
    
#     free_cells = []
#     for y in range(len(map)):
#         for x in range(len(map[0])):
#             if map[y][x] == 0:
#                 free_cells.append([x, y])

#     return random.sample(free_cells, num_agents)

def get_edge_cells(map):
    cells = []
    for y in range(len(map)):
        for x in range(len(map[0])):
            if x in {0, len(map[0]) - 1} or y in {0, len(map) - 1}:
                cells.append([x, y])
    return cells


def get_allowed_border(map):
    rows, cols = len(map), len(map[0])
    visited = [[False for _ in range(cols)] for _ in range(rows)]
    edge_cells = get_edge_cells(map)
    border_cells = [(x, y) for x, y in edge_cells if map[y][x] == 0]
    queue = [(x, y) for x, y in edge_cells if map[y][x] == 1]
    popped = []

    while queue:
        x, y = queue.pop()
        popped.append((x, y))

        for dx, dy in [(-1, 0), (1, 0), (0, -1), (0, 1)]:
            nx, ny = x + dx, y + dy
            if 0 <= nx < cols and 0 <= ny < rows:
                if map[ny][nx] == 1 and not visited[ny][nx]:
                    visited[ny][nx] = True
                    queue.append((nx, ny))

    for popped in popped:
        x, y = popped
        for dx in range(-1, 2):
            for dy in range(-1, 2):
                nx, ny = x + dx, y + dy
                if 0 <= nx < cols and 0 <= ny < rows:
                    if map[ny][nx] == 0:
                        border_cells.append((nx, ny))

    return border_cells


def get_random_agent_along_border_raw(map_name, num_agents, border_width=2):
    with open(map_name) as f:
        lines = f.readlines()
        map_lines = lines[4:] # Skip the first four header lines
        map_lines = [line.strip() for line in map_lines if line.strip()]  # Remove any empty lines
        map = [[0 if c == '.' else 1 for c in line.strip()] for line in map_lines]
    
    free_cells = []
    for y in range(len(map)):
        for x in range(len(map[0])):
            if map[y][x] == 0:
                if x < border_width or x >= len(map[0]) - border_width or y < border_width or y >= len(map) - border_width:
                    free_cells.append([x, y])

    return random.sample(free_cells, num_agents)


def get_random_agent_along_border_floodfill(map_name, num_agents):
    with open(map_name) as f:
        lines = f.readlines()
        map_lines = lines[4:] # Skip the first four header lines
        map_lines = [line.strip() for line in map_lines if line.strip()]  # Remove any empty lines
        map = [[0 if c == '.' else 1 for c in line.strip()] for line in map_lines]
    
    free_cells = get_allowed_border(map)
    return random.sample(free_cells, num_agents)



for i in range(len(NUM_AGENT_LOCS)):
    for experiment_id in range(num_experiments):
        map_name, num_agent_starts = MAP_NAMES[i], NUM_AGENT_LOCS[i]
        # agent_locs = get_random_agent_starts(map_name, num_agent_starts)
        map_ending = map_name.split("/")[-1]
        if map_ending.startswith("den") or map_ending.startswith("mc") or map_ending.startswith("ht") or map_ending.startswith("orz") or map_ending.startswith("AR") or map_ending.startswith("lak") or map_ending.startswith("hrt"):
            agent_locs = get_random_agent_along_border_floodfill(map_name, num_agent_starts)
        elif map_ending in ["custom-19-19.map", "custom-13-13.map", "custom-11-11.map", "maze-32-32-2.map"]:
            agent_locs = get_random_agent_along_border_raw(map_name, num_agent_starts, border_width=2)
        else:
            raise ValueError("Unknown map type for border agent placement.")

        print("\n\nAgent locs:", agent_locs)

        scenario_config = {
            "agents": agent_locs,
            "tasks": [],
            "map": map_name.removeprefix("../maps/"),
            "movement": "FOUR_WAY_MOVEMENT",
            "los": "BRES_LOS"
        }
        with open(SCEN_CONFIG, "w") as f:
            json.dump(scenario_config, f, indent=4)

        method_failed = False
        for method_name, method in zip(method_names, methods):
            input_config = method.copy()
            with open("../solver.json", "w") as f:
                json.dump(input_config, f, indent=4)

            print(f"Running experiment on map {map_name} with {num_agent_starts} agent starts, experiment id {experiment_id}, method {method_name}")
            start_time = time.time()
            result = subprocess.run(["./run", SCEN_CONFIG, "../solver.json"], cwd="/home/srikar/Documents/Research/SearchAndTAPFWithPTC/build", check=True, capture_output=True, text=True)
            # result = subprocess.run(["./run", SCEN_CONFIG, "../solver.json"], cwd="/home/srikar/Documents/Research/SearchAndTAPFWithPTC/build", check=True)
            time_taken = time.time() - start_time
            time_ms = int(time_taken * 1000)

            search_passed = False
            search_time = -1.0
            pd_time = 0.0
            lines = result.stdout.strip().split("\n")
            for line in lines:
                line = line.strip()
                if line.startswith("PD precomputation time:"):
                    pd_time = float(line.split()[-1].strip())
                elif line.startswith("Experiment Search Time:"):
                    search_passed = True
                    search_time = float(line.split()[-1].strip())

            search_time_ms = int((pd_time + search_time) * 1000)

            
            # print("\tMethod", method_name, "expanded", num_expanded, " nodes and took", time_ms, "ms to run")
            print("\tMethod", method_name, " nodes and took", search_time_ms, "ms to run")

            results = open("results/game_map_scaling2.csv", "a+")
            # results = open("results/test.csv", "a+")

  
            results.write(f"{map_name},{method_name},{num_agent_starts},{experiment_id},{search_passed},{search_time_ms}\n")
            results.close()

            # if search_time_ms < 0:
            #     print("Method failed, breaking and writing a line to results to demonstrate failure.")
            #     results.write(f"{map_name},{method_name},{num_agent_starts},{experiment_id},{search_passed},{search_time_ms}\n\n")
            #     results.close()
            #     break
            # else:
            #     results.write(f"{map_name},{method_name},{num_agent_starts},{experiment_id},{search_passed},{search_time_ms}\n")
            #     results.close()


