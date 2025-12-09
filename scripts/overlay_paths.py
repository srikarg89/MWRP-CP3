import json, sys
import numpy as np
from matplotlib import pyplot as plt
import matplotlib.colors
from matplotlib.animation import FuncAnimation
from matplotlib.collections import PatchCollection
import matplotlib.patches as patches

paths = []

# Suboptimal solution
# paths.append([(0,0), (1,0), (2,0), (3,0), (4,0), (5,0), (6,0), (7,0), (8,0), (9,0), (10,0), (11,0), (10,0), (9,0), (8,0), (7,0), (6,0), (5,0), (4,0), (3,0), (2,0), (1,0), (0,0), (0,1), (0,2), (0,3), (0,4), (0,5), (0,6), (1,6), (2,6), (3,6), (4,6), (3,6), (2,6), (1,6), (0,6), (0,5), (0,4), (0,3), (0,2), (0,1), (0,0), (1,0), (2,0), (3,0), (4,0), (5,0), (6,0), (7,0), (8,0), (9,0), (10,0), (11,0), (12,0), (13,0), (12,0), (11,0), (10,0), (9,0), (8,0), (7,0), (6,0), (5,0), (4,0), (3,0), (2,0), (1,0), (0,0), (0,1), (0,2), (1,2), (1,3), (1,4), (1,5), (1,6), (2,6), (3,6), (4,6), (5,6), (4,6), (3,6), (2,6), (1,6), (0,6), (0,5), (0,4), (0,3), (0,2), (0,1), (0,0), (1,0), (2,0), (3,0), (4,0), (5,0), (6,0), (7,0), (8,0), (9,0), (10,0), (11,0), (12,0), (13,0), (13,1), (13,2), (12,2), (11,2), (10,2), (9,2), (8,2), (8,3), (8,4), (8,5), (9,5), (10,5), (11,5), (11,4), (12,4), (13,4), (14,4), (15,4), (16,4), (17,4), (17,5), (18,5), (18,6), (18,7) ])
# paths.append([(0,18), (1,18), (2,18), (3,18), (4,18), (5,18), (4,18), (3,18), (2,18), (1,18), (0,18), (0,17), (0,16), (0,15), (0,14), (0,13), (0,12), (0,11), (0,10), (0,9), (0,8), (1,8), (2,8), (3,8), (2,8), (1,8), (0,8), (0,9), (0,10), (0,11), (0,12), (0,13), (0,14), (0,15), (0,16), (1,16), (2,16), (3,16), (4,16), (4,17), (4,18), (5,18), (6,18), (5,18), (4,18), (3,18), (2,18), (1,18), (0,18), (0,17), (0,16), (0,15), (0,14), (1,14), (2,14), (3,14), (2,14), (1,14), (0,14), (0,15), (0,16), (1,16), (2,16), (3,16), (4,16), (4,17), (4,18), (5,18), (6,18), (7,18), (7,17), (7,16), (7,15), (8,15), (9,15), (10,15), (9,15), (8,15), (7,15), (7,16), (7,17), (7,18), (6,18), (5,18), (4,18), (3,18), (2,18), (1,18), (0,18), (0,17), (0,16), (0,15), (0,14), (1,14), (2,14), (3,14), (4,14), (5,14), (5,13), (6,13), (7,13), (8,13), (9,13), (10,13), (11,13), (10,13), (10,12), (10,11), (11,11), (11,10), (10,10), (11,10), (11,11), (11,12), (11,13), (12,13), (13,13), (14,13), (15,13), (16,13), (17,13), (18,13), (18,14), (18,15), (18,16) ])
# map_state = "1444444444444450003455555555555505333044035033333333533304433555500555555335444050350050300000044555335333055555004400333533305033003555555555555500555043333033333333350504553555555555555353453053333333333533045335333050033053004533555555005555355455550300000000333340003055555555500004555555330333355550444445300533335033344444550050333533331444444445333353333"

# Partition 1
# paths.append([(0,0), (0,1), (0,2), (0,3), (0,4), (0,5), (0,6), (1,6), (2,6), (3,6), (4,6), (5,6), (4,6), (3,6), (2,6), (1,6), (0,6), (0,5), (0,4), (0,3), (0,2), (0,1), (0,0), (1,0), (2,0), (3,0), (4,0), (5,0), (6,0), (7,0), (8,0), (9,0), (10,0), (11,0), (12,0), (13,0), (13,1), (13,2), (12,2), (11,2), (10,2), (9,2), (9,3), (9,4), (9,5), (10,5), (11,5), (11,4), (12,4), (13,4), (14,4), (15,4), (16,4), (17,4), (17,5), (18,5), (18,6) ])
# map_state = "1444444444444450003455555555555505333044035033333333533304433555500555555335444050350050300000044555335333055555004400333533305033003555555555555500555044444444444444454544554555555555555454454454444444444544445445444454444454404544555555445555455455554444444444444444444455555555544444555555444444455554444445444544445444444444554454444544444444444445444454444"

# Partition 2
paths.append([(0,18), (1,18), (2,18), (3,18), (4,18), (5,18), (6,18), (7,18), (7,17), (7,16), (7,15), (8,15), (9,15), (10,15), (9,15), (8,15), (7,15), (7,16), (7,17), (7,18), (6,18), (5,18), (4,18), (3,18), (2,18), (1,18), (0,18), (0,17), (0,16), (0,15), (0,14), (0,13), (0,12), (0,11), (0,10), (0,9), (0,8), (1,8), (2,8), (3,8), (2,8), (1,8), (0,8), (0,9), (0,10), (0,11), (0,12), (0,13), (0,14), (1,14), (2,14), (3,14), (4,14), (5,14), (5,13), (6,13), (7,13), (8,13), (9,13), (10,13), (10,12), (10,11), (10,10), (11,10), (11,11), (11,12), (11,13), (12,13), (13,13), (14,13), (15,13), (15,14), (16,14), (17,14), (18,14), (18,15), (18,16) ])
map_state = "4444444444444454444455555555555545444444445444444444544444444555544555555445444454454454444444444555445444455555444444444544445444444555555555555544555443333033333444350544553555555555555354453053333333333533445335333050033053044533555555005555355455550300000000333340003055555555500004555555330333355550444445300533335033344444550050333533331444444445333353333"


map_width = 19
map_height = 19

map = []
for i in range(0, len(map_state), map_width):
    row_str = map_state[i:i+map_width]
    row = [int(c) / 5 for c in row_str]
    map.append(row)


fig, ax = plt.subplots()
SCALE = 24 / max(map_width, map_height)
fig.set_size_inches(int(map_height * SCALE), int(map_width * SCALE))

colors = ["white","green","white","white","gray","black"]
cmap = matplotlib.colors.LinearSegmentedColormap.from_list("", colors)
im = ax.imshow(map, cmap=cmap)

ax.grid(which='major', axis='both', linestyle='-', color='k', linewidth=1.5)
ax.set_xticks(np.arange(-.5, map_height, 1))
ax.set_yticks(np.arange(-.5, map_width, 1))

ax.set_yticklabels([])
ax.set_xticklabels([])

arrows = {}
for path in paths:
    for i in range(len(path) - 1):
        prev_pos = path[i]
        after_pos = path[i + 1]

        major_offset, minor_offset, length, width = 0.85, 0.1, 0.75, 0.4
        diag_minor_offset = 0.2
        diag_length = .75

        if after_pos == (prev_pos[0] + 1, prev_pos[1]): # Right
            arrows[(prev_pos, after_pos)] = patches.Arrow(prev_pos[0] + minor_offset, prev_pos[1] + minor_offset, length, 0, width=width, color='green')
        elif after_pos == (prev_pos[0] - 1, prev_pos[1]): # Left
            arrows[(prev_pos, after_pos)] = patches.Arrow(prev_pos[0] - minor_offset, prev_pos[1] - minor_offset, -length, 0, width=width, color='green')
        elif after_pos == (prev_pos[0], prev_pos[1] + 1): # Down
            arrows[(prev_pos, after_pos)] = patches.Arrow(prev_pos[0] - minor_offset, prev_pos[1] + minor_offset, 0, length, width=width, color='green')
        elif after_pos == (prev_pos[0], prev_pos[1] - 1): # Up
            arrows[(prev_pos, after_pos)] = patches.Arrow(prev_pos[0] + minor_offset, prev_pos[1] - minor_offset, 0, -length, width=width, color='green')
        elif after_pos == (prev_pos[0] + 1, prev_pos[1] + 1): # Down-Right
            arrows[(prev_pos, after_pos)] = patches.Arrow(prev_pos[0], prev_pos[1] + diag_minor_offset, diag_length, diag_length - diag_minor_offset, width=width, color='green')
        elif after_pos == (prev_pos[0] - 1, prev_pos[1] + 1): # Down-Left
            arrows[(prev_pos, after_pos)] = patches.Arrow(prev_pos[0] - diag_minor_offset, prev_pos[1], -diag_length + diag_minor_offset, diag_length, width=width, color='green')
        elif after_pos == (prev_pos[0] + 1, prev_pos[1] - 1): # Up-Right
            arrows[(prev_pos, after_pos)] = patches.Arrow(prev_pos[0] + diag_minor_offset, prev_pos[1], diag_length - diag_minor_offset, -diag_length, width=width, color='green')
        elif after_pos == (prev_pos[0] - 1, prev_pos[1] - 1): # Up-Left
            arrows[(prev_pos, after_pos)] = patches.Arrow(prev_pos[0], prev_pos[1] - diag_minor_offset, -diag_length, -diag_length + diag_minor_offset, width=width, color='green')


for key, arrow in arrows.items():
    arrow.set_zorder(5)
    arrow.set_alpha(1.0) # Visible by default
    ax.add_patch(arrow)

plt.savefig('../images/Postprocessing 3.png', dpi=500)
# plt.show()