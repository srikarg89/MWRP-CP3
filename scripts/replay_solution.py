import json, sys
import numpy as np
from matplotlib import pyplot as plt
import matplotlib.colors
from matplotlib.animation import FuncAnimation

MAP_NAME = "11_by_11_maze.json"

# Read in watchman_solution.csv
data = []
with open('../build/watchman_solution.csv', 'r') as file:
    next(file)  # Skip header line
    for line in file:
        parts = line.strip().split(',')
        timestep = int(parts[0])
        num_agents = int(parts[1])
        num_seen = int(parts[2])
        poses = [(int(parts[3 + i*2]), int(parts[4 + i*2])) for i in range(num_agents)]
        seen_bitset = parts[3 + num_agents*2]
        data.append((timestep, num_agents, num_seen, poses, seen_bitset))

print("Data length:", len(data))

with open("../configs/" + MAP_NAME) as f:
    config = json.load(f)

agents = config["agents"]
map = config["map"]

fig, ax = plt.subplots()

colors = ["white","green","gray","black"]
cmap = matplotlib.colors.LinearSegmentedColormap.from_list("", colors)
im = ax.imshow(map, cmap=cmap)

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
    # im.set_array(new_data)
    df = data[frame_num]
    timestep, num_agents, num_seen, poses, seen_bitset = df
    arr = [r.copy() for r in map_copy]
    for i in range(len(seen_bitset)):
        row = i // len(map[0])
        col = i % len(map[0])
        if arr[row][col] == 0:  # Only mark non obstacle cells
            arr[row][col] = int(seen_bitset[i]) / (len(colors) - 1)

    for (x, y) in poses:
        arr[y][x] = 1 / (len(colors) - 1)

    im.set_array(arr)

    pos_str = "[" + ", ".join([f"({x},{y})" for (x,y) in poses]) + "]"
    text.set_text(f"Timestep: {timestep}, Num Seen: {num_seen}, Positions: {pos_str}")

    return [im, text] # Return a list of artists that were modified

name = sys.argv[1]
fps = int(sys.argv[2])

print("Creating animation...")
print("Solution length: ", len(data))
num_frames = len(data)
interval_ms = 1000 // fps # 50 milliseconds between frames
anim = FuncAnimation(fig, animate_func, frames=num_frames, interval=interval_ms, blit=True)
anim.save(f'animations/{name}.mp4', writer='ffmpeg', fps=fps)
