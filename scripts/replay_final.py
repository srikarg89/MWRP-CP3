import json, sys
import numpy as np
from matplotlib import pyplot as plt
import matplotlib.colors
from matplotlib.animation import FuncAnimation
from matplotlib.collections import PatchCollection
import matplotlib.patches as patches

# Read in search_solution.csv
data = []
with open('../build/final_solution.csv', 'r') as file:
    next(file)  # Skip header line
    MAP_NAME = "../maps/" + next(file).strip()  # Read map name line
    for line in file:
        parts = line.strip().split(',')
        timestep = int(parts[0])
        map_bitset = parts[1]
        task_bitset = parts[2]
        planned_paths = parts[3:]  # Remaining parts are planned paths
        data.append((timestep, map_bitset, task_bitset, planned_paths))

print("Data length:", len(data))

with open(MAP_NAME) as f:
    lines = f.readlines()
    map_lines = lines[4:] # Skip the first four header lines
    map_lines = [line.strip() for line in map_lines if line.strip()]  # Remove any empty lines
    map = [[0 if c == '.' else 1 for c in line.strip()] for line in map_lines]

fig, ax = plt.subplots()

colors = ["white","green","gray","black"]
cmap = matplotlib.colors.LinearSegmentedColormap.from_list("", colors)
im = ax.imshow(map, cmap=cmap)

task_patches = []
for i in range(len(task_bitset)):
    row = i // len(map[0])
    col = i % len(map[0])
    if task_bitset[i] == '0':
        task_patches.append(None)
        continue
    if task_bitset[i] == '1':
        task_patches.append(patches.Circle((col, row), radius=0.3, color='red'))
    elif task_bitset[i] == '2':
        task_patches.append(patches.Circle((col, row), radius=0.3, color='blue'))
    elif task_bitset[i] == '3':
        task_patches.append(patches.Circle((col, row), radius=0.3, color='gray'))
    else:
        raise ValueError(f"Unknown task bitset value: {task_bitset[i]}")
    ax.add_patch(task_patches[-1])

ax.grid(which='major', axis='both', linestyle='-', color='k', linewidth=1.5)
ax.set_xticks(np.arange(-.5, len(map[0]), 1))
ax.set_yticks(np.arange(-.5, len(map), 1))

ax.set_yticklabels([])
ax.set_xticklabels([])

arrows = {}
for i in range(len(map_bitset)):
    row = i // len(map[0])
    col = i % len(map[0])
    major_offset, minor_offset, length, width = 0.85, 0.1, 0.75, 0.4
    # Right, Left, Up, Down
    arrows[(col, row)] = [
        patches.Arrow(col - major_offset, row + minor_offset, length, 0, width=width, color='green'),
        patches.Arrow(col + major_offset, row - minor_offset, -length, 0, width=width, color='green'),
        patches.Arrow(col - minor_offset, row - major_offset, 0, length, width=width, color='green'),
        patches.Arrow(col + minor_offset, row + major_offset, 0, -length, width=width, color='green'),
    ]
    for arrow in arrows[(col, row)]:
        arrow.set_zorder(5)
        arrow.set_alpha(0.0) # Invisible by default
        ax.add_patch(arrow)


map_copy = [r.copy() for r in map.copy()]

text = plt.text(0, -1, "", fontsize=9)

def animate_func(frame_num):
    print("Animating frame:", frame_num)
    # Generate or load data for the current frame
    df = data[frame_num]
    timestep, map_bitset, task_bitset, planned_paths = df
    arr = [r.copy() for r in map_copy]
    position_str = ""
    for i in range(len(map_bitset)):
        row = i // len(map[0])
        col = i % len(map[0])
        if arr[row][col] == 0:  # Only mark non obstacle cells
            arr[row][col] = int(map_bitset[i]) / (len(colors) - 1)
            if map_bitset[i] == '1':
                position_str += f"({col},{row}) "

    im.set_array(arr)
    text.set_text(f"Timestep: {timestep}, Positions: {position_str}")

    for i in range(len(task_bitset)):
        row = i // len(map[0])
        col = i % len(map[0])
        if task_bitset[i] == '1': # Known Incomplete Task
            task_patches[i].set_alpha(1.0)
            task_patches[i].set_color('purple')
        elif task_bitset[i] == '2': # Completed Task
            task_patches[i].set_alpha(1.0)
            task_patches[i].set_color('blue')
        elif task_bitset[i] == '3': # Unknown Task
            task_patches[i].set_alpha(1.0)
            task_patches[i].set_color('red')
    
    for key, val in arrows.items():
        for arrow in val:
            arrow.set_alpha(0.0) # Hide all arrows initially
    
    for path in planned_paths:
        path = path.split("_")
        for i in range(len(path) - 1):
            if path[i] == path[i + 1] or path[i] == '' or path[i + 1] == '':
                continue
            prev = int(path[i])
            prev_pos = (prev % len(map[0]), prev // len(map[0])) # (col, row) = (x, y)
            after = int(path[i + 1])
            after_pos = (after % len(map[0]), after // len(map[0])) # (col, row) = (x, y)

            # Scale from 1.0 to 0.3
            alpha = 1.0 - (0.7 * (i / (len(path) - 1)))

            if after_pos == (prev_pos[0] + 1, prev_pos[1]):
                arrows[after_pos][0].set_alpha(alpha) # Right
            elif after_pos == (prev_pos[0] - 1, prev_pos[1]):
                arrows[after_pos][1].set_alpha(alpha) # Left
            elif after_pos == (prev_pos[0], prev_pos[1] + 1):
                arrows[after_pos][2].set_alpha(alpha) # Up
            elif after_pos == (prev_pos[0], prev_pos[1] - 1):
                arrows[after_pos][3].set_alpha(alpha) # Down

    
    pat = [p for p in task_patches if p is not None]
    arrow_pats = [a for arrow_list in arrows.values() for a in arrow_list]
    return [im, text, *pat, *arrow_pats] # Return a list of artists that were modified

name = sys.argv[1]
fps = int(sys.argv[2])

print("Creating animation...")
print("Solution length: ", len(data))
num_frames = len(data)
interval_ms = 1000 // fps # 50 milliseconds between frames
anim = FuncAnimation(fig, animate_func, frames=num_frames, interval=interval_ms, blit=True)
anim.save(f'animations/{name}.mp4', writer='ffmpeg', fps=fps)
