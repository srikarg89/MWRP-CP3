import json, sys
import numpy as np
from matplotlib import pyplot as plt
import matplotlib.colors

config_file = sys.argv[1]
with open("../configs/" + config_file) as f:
    config = json.load(f)

agents = config["agents"]
map = config["map"]

fig, ax = plt.subplots()

for x, y in agents:
    map[y][x] = 2

cmap = matplotlib.colors.LinearSegmentedColormap.from_list("", ["white","black","green"])
im = ax.imshow(map, cmap=cmap)

ax.grid(which='major', axis='both', linestyle='-', color='k', linewidth=1.5)
ax.set_xticks(np.arange(-.5, len(map[0]), 1))
ax.set_yticks(np.arange(-.5, len(map), 1))

ax.set_yticklabels([])
ax.set_xticklabels([])

plt.show()