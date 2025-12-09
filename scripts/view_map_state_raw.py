import json, sys
import numpy as np
from matplotlib import pyplot as plt
import matplotlib.colors

names = [
    ("den202d", "../maps/den202d.map"),
    ("den101d", "../maps/den101d.map"),
    ("room", "../maps/room-24-24-4.map"),
    ("lak105d", "../maps/maps2/lak105d.map"),
]

for MAP, MAP_NAME in names:
    with open(MAP_NAME) as f:
        lines = f.readlines()
        map_lines = lines[4:] # Skip the first four header lines
        map_lines = [line.strip() for line in map_lines if line.strip()]  # Remove any empty lines
        map = [[0 if c == '.' else 1 for c in line.strip()] for line in map_lines]

    SCALE = 24 / max(len(map), len(map[0]))

    fig, ax = plt.subplots()
    fig.set_size_inches(int(len(map[0]) * SCALE), int(len(map) * SCALE))

    cmap = matplotlib.colors.LinearSegmentedColormap.from_list("", ["white","black"])
    im = ax.imshow(map, cmap=cmap)

    ax.grid(which='major', axis='both', linestyle='-', color='k', linewidth=4)
    ax.set_xticks(np.arange(-.5, len(map[0]), 1))
    ax.set_yticks(np.arange(-.5, len(map), 1))

    ax.set_yticklabels([])
    ax.set_xticklabels([])

    plt.savefig(f'../images/Map Example {MAP}.png', dpi=500)

    # plt.show()
