import subprocess
import json
import time
import random

SEARCH_TIME = 100.0

def get_template(method, weight):
    if method == "AMWAstar":
        centralized_astar_weight = weight
        centralized_focal_epsilon = 1.0
        focal_method = "SOC"
    elif method == "SOC":
        centralized_astar_weight = 1.0
        centralized_focal_epsilon = weight
        focal_method = "SOC"
    elif method == "MOC":
        centralized_astar_weight = 1.0
        centralized_focal_epsilon = weight
        focal_method = "MOC"
    else:
        raise ValueError("Unknown method: " + method)
    
    return {
        "heuristic": "TSP",
        "focal_method": focal_method,
        "cell_pruning_method": "CELL_THEN_PATH",
        "prune_pivots": True,
        "run_parallel": True,
        "expand_lowest_cost_agent_only": False,
        "max_pivots_generated": 10000000,
        "max_pivots_after_pruning": 10000000,
        "centralized_focal_epsilon": centralized_focal_epsilon,
        "centralized_focal_heuristic_weight": 10000.0,
        "centralized_search_time_limit": SEARCH_TIME,
        "centralized_astar_weight": centralized_astar_weight,
        "run_decentralized_search": False,
        "decentralized_focal_epsilon": 1.0,
        "decentralized_focal_heuristic_weight": 1.5,
        "decentralized_search_time_limit": 10.0,
        "decentralized_astar_weight": 1.0,
        "max_decentralized_searches": 2
    }

methods = [
    get_template("AMWAstar", 10.0),
    get_template("SOC", 10.0),
    # get_template("MOC", 10.0),
    get_template("AMWAstar", 5.0),
    get_template("SOC", 5.0),
    # get_template("MOC", 5.0),
    get_template("AMWAstar", 3.0),
    get_template("SOC", 3.0),
    # get_template("MOC", 3.0),
    get_template("AMWAstar", 2.0),
    get_template("SOC", 2.0),
    # get_template("MOC", 2.0),
]


num_experiments = 1

# CONFIGS = ["../configs/mc_forest_3_robots.json", "../configs/huge/ht_chantry_duo.json", "../configs/mc_forest_4_robots.json", "../configs/huge/ht_chantry_multi.json"]
# CONFIGS = ["../configs/mc_forest_3_robots.json", "../configs/huge/ht_chantry_duo.json"]
CONFIGS = ["../configs/mc_forest.json"]

count = 0
for template in methods:
    for config in CONFIGS:
        if template["centralized_astar_weight"] > 1.0:
            name = f"AMWAstar with weight {template['centralized_astar_weight']}"
        elif template["focal_method"] == "SOC":
            name = f"SOC with epsilon {template['centralized_focal_epsilon']}"
        else:
            name = f"MOC with epsilon {template['centralized_focal_epsilon']}"

        print(f"\n\n\nRunning {name} on config {config}\n\n")
        with open("../solver.json", "w") as f:
            json.dump(template, f, indent=4)

        subprocess.run(["./run", config, "../solver.json"], cwd="/home/srikar/Documents/Research/SearchAndTAPFWithPTC/build", check=True)
