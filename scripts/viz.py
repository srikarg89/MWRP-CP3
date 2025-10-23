import json, sys
import numpy as np
from matplotlib import pyplot as plt
import matplotlib.colors

config_file = sys.argv[1]
with open("../configs/" + config_file) as f:
    config = json.load(f)

agents = config["agents"]
map = config["map"]

with open("../maps/" + map) as f:
    lines = f.readlines()
    map_lines = lines[4:] # Skip the first four header lines
    map_lines = [line.strip() for line in map_lines if line.strip()]  # Remove any empty lines
    map = [[0 if c == '.' else 1 for c in line.strip()] for line in map_lines]


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