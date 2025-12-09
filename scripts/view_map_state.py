import json, sys
import numpy as np
from matplotlib import pyplot as plt
import matplotlib.colors

map_width = int(sys.argv[1])
map_state = sys.argv[2]

map = []
for i in range(0, len(map_state), map_width):
    row_str = map_state[i:i+map_width]
    row = [int(c) / 4 for c in row_str]
    map.append(row)

SCALE = 24 / max(len(map), len(map[0]))

fig, ax = plt.subplots()
fig.set_size_inches(int(len(map[0]) * SCALE), int(len(map) * SCALE))

cmap = matplotlib.colors.LinearSegmentedColormap.from_list("", ["white","green","darkcyan","orange","gray","black"])
im = ax.imshow(map, cmap=cmap)

ax.grid(which='major', axis='both', linestyle='-', color='k', linewidth=1)
ax.set_xticks(np.arange(-.5, len(map[0]), 1))
ax.set_yticks(np.arange(-.5, len(map), 1))

ax.set_yticklabels([])
ax.set_xticklabels([])

# plt.savefig('../images/Path Domination Comparison 11x11 Maze.png', dpi=500)
# plt.savefig('../images/Cell Domination Comparison 32x32 Maze.png', dpi=500)
# plt.savefig('../images/Cell Domination Comparison MC Map.png', dpi=500)
# plt.savefig('../images/Cell Domination Comparison HT Chantry 1.png', dpi=500)
# plt.savefig('../images/Path Domination Comparison Den101d 2.png', dpi=500)
# plt.savefig('../images/32x32 Map Example.png', dpi=500)

# plt.savefig('../images/32x32 Cell and Path Domination.png', dpi=500)
# plt.savefig('../images/HT Chantry Cell and Path Domination.png', dpi=500)
# plt.savefig('../images/Minecraft Cell and Path Domination 3 Agents.png', dpi=500)
plt.savefig('../images/Random Cell and Path Domination 1 Agent.png', dpi=500)

plt.show()
