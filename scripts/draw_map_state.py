import json, sys
import numpy as np
from matplotlib import pyplot as plt
import matplotlib.colors
import matplotlib.patheffects as pe

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

# cmap = matplotlib.colors.LinearSegmentedColormap.from_list("", ["white","green","purple","yellow","black"])
# map = [
#     [1, 1, 1, 3/4, 3/4, 3/4, 3/4],
#     [1, 1, 0, 0, 1, 3/4, 3/4],
#     [1, 1, 0, 1, 1, 1, 1],
#     [1/4, 0, 0, 1, 1, 1, 1],
# ]

# map = [
#     [1, 1, 1, 3/4, 3/4, 3/4, 3/4],
#     [1, 1, 3/4, 0, 1, 3/4, 3/4],
#     [1, 1, 0, 1, 1, 1, 1],
#     [1/4, 0, 0, 1, 1, 1, 1],
# ]

# map = [
#     [1, 1, 1, 0, 0, 0, 2/4],
#     [1, 1, 2/4, 0, 1, 0, 0],
#     [1, 1, 0, 1, 1, 1, 1],
#     [1/4, 0, 0, 1, 1, 1, 1],
# ]

# MWA* example
cmap = matplotlib.colors.LinearSegmentedColormap.from_list("", ["white","green","gray","black"])
map = [
    [2/3, 0, 0, 0, 0, 0, 0],
    [2/3, 1, 1, 1, 1, 1, 0],
    [2/3, 1, 0, 0, 0, 1, 0],
    [1/3, 1, 0, 1, 1, 1, 0],
    [1/3, 1, 0, 0, 0, 0, 0],
]

fig, ax = plt.subplots()


im = ax.imshow(map, cmap=cmap)

ax.grid(which='major', axis='both', linestyle='-', color='k', linewidth=3.0)
ax.set_xticks(np.arange(-.5, len(map[0]), 1))
ax.set_yticks(np.arange(-.5, len(map), 1))

ax.set_yticklabels([])
ax.set_xticklabels([])

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

plt.show()