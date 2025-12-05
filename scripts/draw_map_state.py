import json, sys
import numpy as np
from matplotlib import pyplot as plt
import matplotlib.colors
import matplotlib.patheffects as pe
import matplotlib.patches as patches

# cmap = matplotlib.colors.LinearSegmentedColormap.from_list("", ["white","green","darkcyan","orange","gray","black"])
# cmap = matplotlib.colors.LinearSegmentedColormap.from_list("", ["white","green","gray","gray","gray","black"])
# cmap = matplotlib.colors.LinearSegmentedColormap.from_list("", ["white","green","darkcyan","orange","gray","black"])

# map = [
#     [1/3,0,  0, 0, 0, 0],
#     [1, 1, 0, 1, 1, 1],
#     [1, 1, 0, 2/3, 1, 1],
#     [1, 1, 0, 1, 1, 1],
#     [1, 0, 0, 0, 0, 0]
# ]

# map2 = [
#     [1/3, 0, 0, 0, 0],
#     [1, 0, 1, 1, 1],
#     [1, 2/3, 1, 1, 1],
#     [1, 0, 1, 1, 1],
#     [1, 0, 0, 0, 0]
# ]

# map3 = [
#     [1/3, 0, 0, 0, 0],
#     [1, 3/4, 1, 1, 1],
#     [1, 1/4, 1, 1, 1],
#     [1, 0, 1, 1, 1],
#     [1, 0, 0, 0, 0]
# ]

# Cell / Path domination maps

cmap = matplotlib.colors.LinearSegmentedColormap.from_list("", ["white","green","darkorchid","yellow","black"])
map = [
    [1, 1, 1, 3/4, 3/4, 3/4, 2/4],
    [1, 1, 0, 0, 1, 3/4, 3/4],
    [1, 1, 0, 1, 1, 1, 1],
    [0, 0, 0, 1, 1, 1, 1],
]

map = [
    [1, 1, 1, 3/4, 3/4, 2/4, 3/4],
    [1, 1, 3/4, 0, 1, 3/4, 3/4],
    [1, 1, 0, 1, 1, 1, 1],
    [0, 0, 0, 1, 1, 1, 1],
]

# map = [
#     [1, 1, 1, 0, 0, 0, 2/4],
#     [1, 1, 2/4, 0, 1, 0, 0],
#     [1, 1, 0, 1, 1, 1, 1],
#     [1/4, 0, 0, 1, 1, 1, 1],
# ]

# MWA* example
# cmap = matplotlib.colors.LinearSegmentedColormap.from_list("", ["white","green","gray","black"])
# map = [
#     [2/3, 0, 0, 0, 0, 0, 0],
#     [2/3, 1, 1, 1, 1, 1, 0],
#     [2/3, 1, 0, 0, 0, 1, 0],
#     [1/3, 1, 0, 1, 1, 1, 0],
#     [1/3, 1, 0, 0, 0, 0, 0],
# ]

# # Pivot pruning
# cmap = matplotlib.colors.LinearSegmentedColormap.from_list("", ["white","green","gray","firebrick","yellow","black"])
# map = [
#     [1/5, 2/5, 2/5, 2/5, 2/5, 2/5],
#     [1, 1, 1, 1, 1, 4/5],
#     [1, 4/5, 4/5, 3/5, 1, 4/5],
#     [1, 0, 1, 1, 1, 4/5],
#     [1, 4/5, 4/5, 4/5, 4/5, 3/5],
# ]

# # map = [
# #     [1/5, 2/5, 2/5, 2/5, 2/5, 2/5],
# #     [1, 1, 1, 1, 1, 0],
# #     [1, 4/5, 4/5, 3/5, 1, 0],
# #     [1, 0, 1, 1, 1, 0],
# #     [1, 0, 0, 0, 0, 0],
# # ]

# BresLOS
# cmap = matplotlib.colors.LinearSegmentedColormap.from_list("", ["white","green","gray","black"])
# map = [
#     [2/3, 2/3, 2/3, 2/3, 2/3],
#     [1, 1, 1/3, 1, 1],
#     [2/3, 2/3, 2/3, 2/3, 2/3],
#     [2/3, 1, 1, 1, 2/3],
#     [0, 1, 0, 0, 0],
# ]

# Movement
# cmap = matplotlib.colors.LinearSegmentedColormap.from_list("", ["white","green"])
# map = [
#     [0, 0, 0],
#     [0, 1, 0],
#     [0, 0, 0],
# ]

# Heuristic
# cmap = matplotlib.colors.LinearSegmentedColormap.from_list("", ["white","green","gray","red","yellow","black"])
# map = [
#     [4/5, 4/5, 4/5, 3/5],
#     [0, 1, 1, 1],
#     [4/5, 4/5, 4/5, 3/5],
#     [0, 1, 1, 1],
#     [2/5, 2/5, 2/5, 1/5],
# ]

fig, ax = plt.subplots()

im = ax.imshow(map, cmap=cmap)

ax.grid(which='major', axis='both', linestyle='-', color='k', linewidth=3.0)
ax.set_xticks(np.arange(-.5, len(map[0]), 1))
ax.set_yticks(np.arange(-.5, len(map), 1))

ax.set_yticklabels([])
ax.set_xticklabels([])

# pairs = [
#     ((1,1), (1,0)),
#     ((1,1), (0,1)),
#     ((1,1), (1,2)),
#     ((1,1), (2,1)),
# ]

# for prev_pos, after_pos in pairs:
#     major_offset, minor_offset, length, width = 0.5, 0.0, 1.0, 0.4
#     diag_minor_offset = 0.2
#     diag_length = .75

#     if after_pos == (prev_pos[0] + 1, prev_pos[1]): # Right
#         arrow = patches.Arrow(prev_pos[0] + minor_offset, prev_pos[1] + minor_offset, length, 0, width=width, color='black')
#     elif after_pos == (prev_pos[0] - 1, prev_pos[1]): # Left
#         arrow = patches.Arrow(prev_pos[0] - minor_offset, prev_pos[1] - minor_offset, -length, 0, width=width, color='black')
#     elif after_pos == (prev_pos[0], prev_pos[1] + 1): # Down
#         arrow = patches.Arrow(prev_pos[0] - minor_offset, prev_pos[1] + minor_offset, 0, length, width=width, color='black')
#     elif after_pos == (prev_pos[0], prev_pos[1] - 1): # Up
#         arrow = patches.Arrow(prev_pos[0] + minor_offset, prev_pos[1] - minor_offset, 0, -length, width=width, color='black')
#     elif after_pos == (prev_pos[0] + 1, prev_pos[1] + 1): # Down-Right
#         arrow = patches.Arrow(prev_pos[0], prev_pos[1] + diag_minor_offset, diag_length, diag_length - diag_minor_offset, width=width, color='green')
#     elif after_pos == (prev_pos[0] - 1, prev_pos[1] + 1): # Down-Left
#         arrow = patches.Arrow(prev_pos[0] - diag_minor_offset, prev_pos[1], -diag_length + diag_minor_offset, diag_length, width=width, color='green')
#     elif after_pos == (prev_pos[0] + 1, prev_pos[1] - 1): # Up-Right
#         arrow = patches.Arrow(prev_pos[0] + diag_minor_offset, prev_pos[1], diag_length - diag_minor_offset, -diag_length, width=width, color='green')
#     elif after_pos == (prev_pos[0] - 1, prev_pos[1] - 1): # Up-Left
#         arrow = patches.Arrow(prev_pos[0], prev_pos[1] - diag_minor_offset, -diag_length, -diag_length + diag_minor_offset, width=width, color='green')
    
#     arrow.set_zorder(5)
#     ax.add_patch(arrow)

# plt.text(
#     5.75, 0.2, "s1",
#     color="white",
#     fontsize=25,
#     path_effects=[
#         pe.withSimplePatchShadow(offset=(3, -3), shadow_rgbFace=(0,0,0), alpha=0.2),
#         # pe.Normal()
#         pe.withStroke(linewidth=2, foreground="black")
#     ]
# )

plt.savefig('../images/ExCellDom2.png', dpi=500, bbox_inches='tight')
plt.show()