import sys
from matplotlib import pyplot as plt

plt.rcParams["axes.labelsize"] = 13       # x/y labels
plt.rcParams["xtick.labelsize"] = 11      # x ticks
plt.rcParams["ytick.labelsize"] = 11      # y ticks
plt.rcParams["legend.fontsize"] = 13      # legend
plt.rcParams["figure.titlesize"] = 16     # figure title (optional)


filenames = [
    "results/maze_map_scaling.csv",
    "results/room_map_scaling_merged.csv",
    "results/random_map_scaling_merged.csv",
    "results/game_map_scaling_merged.csv",

    "results/maze_agent_scaling.csv",
    "results/room_agent_scaling.csv",
    "results/random_agent_scaling.csv",
    "results/game_agent_scaling.csv",
]

types = [
    "MAP", "MAP", "MAP", "MAP", "AGENT", "AGENT", "AGENT", "AGENT"
]

titles = [
    "Maze Maps, 2 Agents",
    "Room Maps, 2 Agents",
    "Random Maps, 2 Agents",
    "Game Maps, 2 Agents",
    "13x13 Maze Map",
    "16x16 Room Map",
    "17x17 Random Map",
    "Lak105d Game Map",
]

x_labels = [
    ['11x11', '13x13', '19x19', '32x32'],
    ['12x16', '16x16', '20x20', '24x24'],
    ['10x10', '13x13', '17x17', '20x20'],
    ['Den202d', 'Lak105d', 'Den101d', 'Minecraft'],
    ['1', '2', '3', '4', '5', '6'],
    ['1', '2', '3', '4', '5', '6'],
    ['1', '2', '3', '4', '5', '6'],
    ['1', '2', '3', '4', '5', '6'],
]

ALG_NAMES = [
    ("OG_MWRP", "MWRP-A*", "blue"),
    ("MWRP_CPD", "MWRP-A* + CPD", "orange"),
    ("MWRP_CP3", "MWRP-CP$^3$", "green"),
    ("MxWAstar", "MxWA*", "red"),
    ("FOCAL_SOC", "FS (SORC)", "purple"),
    ("FOCAL_MOC", "FS (MORC)", "gray"),
]

fig, axes = plt.subplots(2, 4, figsize=(16, 8), subplot_kw={'aspect': 'equal'}, gridspec_kw={'hspace': 0.35})

for i, (filename, TYPE, title) in enumerate(zip(filenames, types, titles)):
    file = open(filename)

    num_agents_set = set()
    map_names = []

    data = {}

    lines = file.readlines()

    for line in lines:
        if line.strip() == "":
            continue
        line = line.strip().split(",")
        map_name, alg, num_agents, experiment_id, solved, time = line
        if int(time) < 0 or not solved:
            print("NOT SOLVED", line)
            continue
        
        if map_name not in map_names:
            map_names.append(map_name)

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


    if "FOCAL_SOC2" in data:
        data["FOCAL_SOC"] = data["FOCAL_SOC2"]

    if "FOCAL_MOC2" in data:
        data["FOCAL_MOC"] = data["FOCAL_MOC2"]

    if "MxWAstar2" in data:
        data["MxWAstar"] = data["MxWAstar2"]

    ax = axes[i // 4, i % 4]
    for alg_name, alg_label, color in ALG_NAMES:
        x = []
        y = []
        if TYPE == "AGENT":
            print("AGENT COUNT", alg_name)
            for num_agents in sorted(num_agents_set):
                times = data[alg_name][num_agents]
                times = sorted(times)[1:-1] if len(times) > 2 else times
                avg_time = sum(times) / len(times)
                x.append(str(num_agents))
                y.append(avg_time)
        else:
            print("MAP SIZE", alg_name)
            for x_label, map_name in zip(x_labels[i], map_names):
                if map_name not in data[alg_name]:
                    continue
                times = data[alg_name][map_name]
                times = sorted(times)[1:-1] if len(times) > 2 else times
                avg_time = sum(times) / len(times)
                x.append(x_label)
                y.append(avg_time)

        if i == 0:
            ax.plot(x, y, label=alg_label, color=color)
        else:
            ax.plot(x, y, color=color)
        
        ax.scatter(x, y, color=color, s=10)


    ax.set_yscale('log')
    ax.set_xlabel('Number of Agents' if TYPE == "AGENT" else 'Map Size')
    if i == 0 or i == 4:
        ax.set_ylabel('Average Runtime (ms)')
    ax.set_aspect('auto')

    ax.set_yticks([10, 100, 1000, 10000, 100000])
    ax.set_yticklabels(['$10$', '$10^2$', '$10^3$', '$10^4$', '$10^5$'])
    ax.tick_params(axis='y', which='minor', length=0)
    ax.set_ylim(8, 200000)
    ax.set_title(title)


fig.legend(loc='lower center', bbox_to_anchor=(0.5, -0.01), ncol=len(ALG_NAMES), frameon=True, edgecolor='black')
plt.tight_layout(rect=[0, 0.05, 1, 1])
# plt.tight_layout()
plt.savefig('all_experiments.png', dpi=500, bbox_inches='tight')
# plt.show()
