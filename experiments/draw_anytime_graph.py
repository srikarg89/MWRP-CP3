import sys
import numpy as np
from matplotlib import pyplot as plt

plt.rcParams["axes.labelsize"] = 22       # x/y labels
plt.rcParams["xtick.labelsize"] = 18      # x ticks
plt.rcParams["ytick.labelsize"] = 18      # y ticks
plt.rcParams["legend.fontsize"] = 17      # legend
plt.rcParams["figure.titlesize"] = 22     # figure title (optional)
name_mapping = {
    "MxWAstar2": "AMWA*, w=2",
    "SOC_2": "AFS (SORC), w=2",
    "MOC_2": "AFS (MORC), w=2",
    "MxWAstar3": "AMWA*, w=3",
    "SOC_3": "AFS (SORC), w=3",
    "MOC_3": "AFS (MORC), w=3",
}

color_mapping = {
    "MxWAstar2": "orange",
    "SOC_2": "deeppink",
    "MOC_2": "blue",
    "MxWAstar3": "red",
    "SOC_3": "purple",
    "MOC_3": "gray",
}

file = open("results/anytime_test.csv")
lines = file.readlines()
data = {}
for line in lines:
    map, alg, sec, cost = line.strip().split(",")
    sec = float(sec)
    cost = int(cost)

    if alg not in data:
        data[alg] = []
    
    data[alg].append((sec, cost))

for alg, alg_data in data.items():
    alg_data.sort()
    times = [t for t, c in alg_data]
    costs = [c for t, c in alg_data]
    if alg == "SOC_2":
        times = [t + 3 for t in times]
    times.append(600)
    costs.append(costs[-1])
    plt.plot(times, costs, label=name_mapping[alg], lw=3, color=color_mapping[alg])

plt.xlim(0, 250)

plt.xlabel("Time (s)")
plt.ylabel("Solution Cost")
plt.title("Minecraft Map, 2 Agents", fontsize=22)


# plt.legend(loc='center right', bbox_to_anchor=(1.471, 0.774), ncol=1, frameon=True, edgecolor='black')
# plt.legend(loc='center right', bbox_to_anchor=(1, 0.775), ncol=2, frameon=True, edgecolor='black')

# plt.legend(loc='center right', bbox_to_anchor=(1.25, -0.25), ncol=3, frameon=True, edgecolor='black')
plt.legend(loc='center right', bbox_to_anchor=(1.08, -0.35), ncol=2, frameon=True, edgecolor='black')

# plt.legend()
# plt.legend(loc='upper right', ncol=2, frameon=True, edgecolor='black')
plt.savefig('anytime_graph.png', bbox_inches='tight', pad_inches=0.05, dpi=500)
# plt.show()
