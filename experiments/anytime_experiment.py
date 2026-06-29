import subprocess
import json
import time
import random

import time
import os

import psutil

W_VALUE = 2.0
TIME_LIMIT = 250.0
PARALLEL_BATCH_SIZE = 100

MWRCP3_TEMPLATE = {
    "heuristic": "LAZY",
    "focal_method": "SOC",
    "cell_pruning_method": "CELL_THEN_PATH",
    "prune_pivots": True,
    "run_parallel": True,
    "max_pivots_generated": 10000000,
    "parallel_batch_size": PARALLEL_BATCH_SIZE,
    "centralized_focal_epsilon": 1.0,
    "centralized_search_time_limit": TIME_LIMIT,
    "centralized_hard_search_time_limit": TIME_LIMIT,
    "centralized_astar_weight": 1.0,
    "run_decentralized_search": False,
    "decentralized_focal_epsilon": 1.0,
    "decentralized_search_time_limit": 10.0,
    "decentralized_hard_search_time_limit": 500.0,
    "decentralized_astar_weight": 1.0,
    "max_decentralized_searches": 2
}

MxWAstar_TEMPLATE = MWRCP3_TEMPLATE.copy()
MxWAstar_TEMPLATE["centralized_astar_weight"] = W_VALUE

FOCAL_SOC_TEMPLATE = MWRCP3_TEMPLATE.copy()
FOCAL_SOC_TEMPLATE["focal_method"] = "SOC"
FOCAL_SOC_TEMPLATE["centralized_focal_epsilon"] = W_VALUE

FOCAL_MOC_TEMPLATE = MWRCP3_TEMPLATE.copy()
FOCAL_MOC_TEMPLATE["focal_method"] = "MOC"
FOCAL_MOC_TEMPLATE["centralized_focal_epsilon"] = W_VALUE


####### SUBOPTIMAL EXPERIMENT CONFIGURATION ########
MxWAsta5_TEMPLATE = MxWAstar_TEMPLATE.copy(); MxWAsta5_TEMPLATE["centralized_astar_weight"] = 5.0
MxWAsta3_TEMPLATE = MxWAstar_TEMPLATE.copy(); MxWAsta3_TEMPLATE["centralized_astar_weight"] = 3.0
MxWAsta2_TEMPLATE = MxWAstar_TEMPLATE.copy(); MxWAsta2_TEMPLATE["centralized_astar_weight"] = 2.0

FOCAL_SOC5_TEMPLATE = FOCAL_SOC_TEMPLATE.copy(); FOCAL_SOC5_TEMPLATE["centralized_focal_epsilon"] = 5.0
FOCAL_SOC3_TEMPLATE = FOCAL_SOC_TEMPLATE.copy(); FOCAL_SOC3_TEMPLATE["centralized_focal_epsilon"] = 3.0
FOCAL_SOC2_TEMPLATE = FOCAL_SOC_TEMPLATE.copy(); FOCAL_SOC2_TEMPLATE["centralized_focal_epsilon"] = 2.0

FOCAL_MOC5_TEMPLATE = FOCAL_MOC_TEMPLATE.copy(); FOCAL_MOC5_TEMPLATE["centralized_focal_epsilon"] = 5.0
FOCAL_MOC3_TEMPLATE = FOCAL_MOC_TEMPLATE.copy(); FOCAL_MOC3_TEMPLATE["centralized_focal_epsilon"] = 3.0
FOCAL_MOC2_TEMPLATE = FOCAL_MOC_TEMPLATE.copy(); FOCAL_MOC2_TEMPLATE["centralized_focal_epsilon"] = 2.0

method_names = [
    "MxWAstar3", "SOC_3", "MOC_3",
    "MxWAstar2", "SOC_2", "MOC_2",
]
methods = [
    MxWAsta3_TEMPLATE, FOCAL_SOC3_TEMPLATE, FOCAL_MOC3_TEMPLATE,
    MxWAsta2_TEMPLATE, FOCAL_SOC2_TEMPLATE, FOCAL_MOC2_TEMPLATE,
]

method_names = [
    "MxWAstar5", "SOC_5", "MOC_5",
]
methods = [
    MxWAsta5_TEMPLATE, FOCAL_SOC5_TEMPLATE, FOCAL_MOC5_TEMPLATE,
]


CONFIG_FILE = "../configs/mc_forest.json"
results_file = "anytime_test2"


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




method_failed = False
for method_name, method in zip(method_names, methods):
    input_config = method.copy()
    with open("../solver.json", "w") as f:
        json.dump(input_config, f, indent=4)

    print(f"Running experiment on config {CONFIG_FILE} with method {method_name}")
    start_time = time.time()
    result = subprocess.run(["./run", CONFIG_FILE, "../solver.json"], cwd="/home/srikar/Documents/Research/SearchAndTAPFWithPTC/build", check=True, capture_output=True, text=True)

    time_taken = time.time() - start_time
    time_ms = int(time_taken * 1000)

    search_passed = False
    search_time = -1.0
    pd_time = 0.0
    solution_quality = -1
    lines = result.stdout.strip().split("\n")
    for line in lines:
        line = line.strip()
        if line.startswith("ANYTIME SOLUTION"):
            line = line.split()
            print(line)
            time_s = float(line[-2].strip())
            solution_quality = int(line[-1].strip())

            print(f"\tSolution found at time {time_s} with quality {solution_quality}")
            results = open(f"results/{results_file}.csv", "a+")
            results.write(f"{CONFIG_FILE},{method_name},{time_s},{solution_quality}\n")
            results.close()
