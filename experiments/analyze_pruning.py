import statistics

# file = open("results/border_test.csv")
file = open("../build/pruning_experiment_updated.csv")
lines = file.readlines()
data = {}

exp_ids_count = {}
avg_unseen_size = {}
for line in lines:
    if len(line.strip()) == 0 or line.startswith("Map,"):
        continue
    line = line.strip().split(",")

    map_name, experiment_id, alg, unseen, original_unseen, time = line
    pruned = int(original_unseen) - int(unseen)
    prune_percentage = pruned / int(original_unseen) * 100.0

    alg = alg.strip()

    if map_name not in data:
        data[map_name] = {}
        exp_ids_count[map_name] = {}
        avg_unseen_size[map_name] = []
    
    if alg not in data[map_name]:
        data[map_name][alg] = {}
        exp_ids_count[map_name][alg] = {}
    
    if experiment_id not in exp_ids_count[map_name][alg]:
        exp_ids_count[map_name][alg][experiment_id] = 0
    
    exp_ids_count[map_name][alg][experiment_id] += 1
    num_agents = exp_ids_count[map_name][alg][experiment_id]

    if num_agents not in data[map_name][alg]:
        data[map_name][alg][num_agents] = [[], []]
    
    data[map_name][alg][num_agents][0].append(prune_percentage)
    data[map_name][alg][num_agents][1].append(float(time))
    avg_unseen_size[map_name].append(int(original_unseen))
    

algs = ["CELL", "PATH", "CELL_THEN_PATH"]

for map_name in sorted(data):
    print(f"Average size of unseen on {map_name}:", sum(avg_unseen_size[map_name]) / len(avg_unseen_size[map_name]))
    for alg in algs:
        # for num_agents in sorted(data[map_name][alg]):
        #     print("Map:", map_name, "Alg:", alg, "Num agents:", num_agents)
        #     print("\tAvg pruned:", sum(data[map_name][alg][num_agents][0]) / len(data[map_name][alg][num_agents][0]))
        #     print("\tAvg time (ms):", sum(data[map_name][alg][num_agents][1]) / len(data[map_name][alg][num_agents][1]))

        stats_data_u = []
        stats_data_t = []
        for num_agents in sorted(data[map_name][alg]):
            stats_data_u.extend(data[map_name][alg][num_agents][0])
            stats_data_t.extend(data[map_name][alg][num_agents][1])
        
        print("Map:", map_name, "Alg:", alg)
        mean_u = sum(stats_data_u) / len(stats_data_u)
        mean_t = sum(stats_data_t) / len(stats_data_t)
        std_u = statistics.stdev(stats_data_u) if len(stats_data_u) > 1 else 0.0
        std_t = statistics.stdev(stats_data_t) if len(stats_data_t) > 1 else 0.0

        mean_u, std_u, mean_t, std_t = round(mean_u, 1), round(std_u, 1), round(mean_t), round(std_t)

        print(f"\tOverall avg pruned: {mean_u} +/- {std_u}")
        print(f"\tOverall avg time (ms): {mean_t} +/- {std_t}")