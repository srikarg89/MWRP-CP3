import statistics

# file = open("results/border_test.csv")
file = open("results/den207_ablation_new2.csv")

alg_names = set()
num_agents_set = set()
map_names = set()

data = {}

lines = file.readlines()
for line in lines:
    if len(line.strip()) == 0:
        continue
    line = line.strip().split(",")
    map_name, alg, num_agents, experiment_id, solved, time = line
    if int(time) < 0:
        print("NOT SOLVED", line)
        continue

    if not solved:
        print("NOT SOLVED:", line)

    map_names.add(map_name)
    alg_names.add(alg)
    num_agents = int(num_agents)
    num_agents_set.add(num_agents)
    if map_name not in data:
        data[map_name] = {}

    if num_agents not in data[map_name]:
        data[map_name][num_agents] = {}

    if experiment_id not in data[map_name][num_agents]:
        data[map_name][num_agents][experiment_id] = {}
        
    data[map_name][num_agents][experiment_id][alg] = int(time)


for map_name in sorted(list(map_names)):
    for num_agents in sorted(data[map_name]):
        no_pp_factors = []
        no_phc_factors = []
        for experiment_id in data[map_name][num_agents]:
            if set(data[map_name][num_agents][experiment_id].keys()) != {"MWRP_CP3", "MWRP_CPD_PP", "MWRP_CPD_PHC"}:
                print("MISSING DATA FOR", map_name, experiment_id, data[map_name][num_agents][experiment_id].keys())
                continue

            base_time = data[map_name][num_agents][experiment_id]["MWRP_CP3"]
            no_pp_time = data[map_name][num_agents][experiment_id]["MWRP_CPD_PHC"]
            no_phc_time = data[map_name][num_agents][experiment_id]["MWRP_CPD_PP"]

            if base_time != min([base_time, no_pp_time, no_phc_time]):
                continue

            no_pp_factors.append(no_pp_time / base_time)
            no_phc_factors.append(no_phc_time / base_time)
    
        avg_no_pp = sum(no_pp_factors) / len(no_pp_factors)
        avg_no_phc = sum(no_phc_factors) / len(no_phc_factors)
        std_no_pp = statistics.stdev(no_pp_factors) if len(no_pp_factors) > 1 else 0.0
        std_no_phc = statistics.stdev(no_phc_factors) if len(no_phc_factors) > 1 else 0.0
        print(f"Map: {map_name}, Num Agents: {num_agents}, No PP Factor: {avg_no_pp:.2f} +/- {std_no_pp:.2f}, Avg No PHC Factor: {avg_no_phc:.2f} +/- {std_no_phc:.2f}")
        print("\tNo PP Factors:", [f"{factor:.2f}" for factor in no_pp_factors])
        print("\tNo PHC Factors:", [f"{factor:.2f}" for factor in no_phc_factors])