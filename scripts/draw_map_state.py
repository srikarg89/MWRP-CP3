import json, sys
import numpy as np
from matplotlib import pyplot as plt
import matplotlib.colors

map = [
    [1/3,0,  0, 0, 0, 0],
    [1, 1, 0, 1, 1, 1],
    [1, 1, 0, 2/3, 1, 1],
    [1, 1, 0, 1, 1, 1],
    [1, 0, 0, 0, 0, 0]
]

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


fig, ax = plt.subplots()

# cmap = matplotlib.colors.LinearSegmentedColormap.from_list("", ["white","green","darkcyan","orange","gray","black"])
# cmap = matplotlib.colors.LinearSegmentedColormap.from_list("", ["white","green","gray","gray","gray","black"])
# cmap = matplotlib.colors.LinearSegmentedColormap.from_list("", ["white","green","darkcyan","orange","gray","black"])
cmap = matplotlib.colors.LinearSegmentedColormap.from_list("", ["white","green","orange","black"])
im = ax.imshow(map, cmap=cmap)

ax.grid(which='major', axis='both', linestyle='-', color='k', linewidth=1.5)
ax.set_xticks(np.arange(-.5, len(map[0]), 1))
ax.set_yticks(np.arange(-.5, len(map), 1))

ax.set_yticklabels([])
ax.set_xticklabels([])

plt.show()