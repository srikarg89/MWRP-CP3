from matplotlib import pyplot as plt

# file = open("main_experiment_results_NEW.csv")
file = open("main_experiment_results_11by11_TSP.csv")
# file = open("maze_map_scaling_results.csv")

# ../maps/custom-11-11.map,MWRP_CPD,6,4,True,1448

# TYPE = "MAP"
TYPE = "AGENT_COUNT"

alg_names = set()
num_agents_set = set()
map_names = set()

data = {}

lines = file.readlines()
for line in lines:
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
    if alg not in data:
        data[alg] = {}

    if num_agents not in data[alg]:
        data[alg][num_agents] = []
    data[alg][num_agents].append(int(time))

    if map_name not in data[alg]:
        data[alg][map_name] = []
    data[alg][map_name].append(int(time))


print(data.keys())


for alg_name in sorted(alg_names):
    x = []
    y = []
    if TYPE == "AGENT_COUNT":
        print("AGENT COUNT", alg_name)
        for num_agents in sorted(num_agents_set):
            times = data[alg_name][num_agents]
            avg_time = sum(times) / len(times)
            print(avg_time)
            x.append(num_agents)
            y.append(avg_time)
    else:
        print("MAP SIZE", alg_name)
        for map_name in sorted(map_names):
            if map_name not in data[alg_name]:
                continue
            times = data[alg_name][map_name]
            avg_time = sum(times) / len(times)
            print(avg_time)
            x.append(map_name)
            y.append(avg_time)
    plt.plot(x, y, label=alg_name)


# plt.gca().set(title='semilogy')
plt.gca().set_yscale('log')


plt.legend()
plt.show()
