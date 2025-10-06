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
        data.append((timestep, map_bitset, task_bitset))

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

map_copy = [r.copy() for r in map.copy()]

text = plt.text(0, -1, "", fontsize=9)

def animate_func(frame_num):
    print("Animating frame:", frame_num)
    # Generate or load data for the current frame
    df = data[frame_num]
    timestep, map_bitset, task_bitset = df
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
            task_patches[i].set_color('red')
        elif task_bitset[i] == '2': # Completed Task
            task_patches[i].set_alpha(1.0)
            task_patches[i].set_color('blue')
        elif task_bitset[i] == '3': # Unknown Task
            task_patches[i].set_alpha(1.0)
            task_patches[i].set_color('gray')
    
    pat = [p for p in task_patches if p is not None]
    return [im, text, *pat] # Return a list of artists that were modified

name = sys.argv[1]
fps = int(sys.argv[2])

print("Creating animation...")
print("Solution length: ", len(data))
num_frames = len(data)
interval_ms = 1000 // fps # 50 milliseconds between frames
anim = FuncAnimation(fig, animate_func, frames=num_frames, interval=interval_ms, blit=True)
anim.save(f'animations/{name}.mp4', writer='ffmpeg', fps=fps)
