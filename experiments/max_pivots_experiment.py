import subprocess
import json

template = {
    "heuristic": "TSP",
    "focal_method": "SOC",
    "prune_pivots": False,
    "run_parallel": True,
    "max_pivots_generated": 8,
    "centralized_focal_epsilon": 1.0,
    "centralized_search_time_limit": 1.0,
    "centralized_astar_weight": 1.0,
    "run_decentralized_search": False,
    "decentralized_focal_epsilon": 1.0,
    "decentralized_search_time_limit": 10.0,
    "decentralized_astar_weight": 1.0,
    "max_decentralized_searches": 2
}

for p in range(6, 15):
    config = template.copy()
    config["max_pivots_generated"] = p
    with open("../solver.json", "w") as f:
        json.dump(config, f, indent=4)
    print(f"\n\n\n\n\nRunning experiment with max_pivots_generated = {p}")
    subprocess.run(["./run", "../configs/big_maze_tight.json", "../solver.json"], cwd="/home/srikar/Documents/Research/SearchAndTAPFWithPTC/build", check=True)
