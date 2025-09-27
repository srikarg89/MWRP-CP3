import json, sys
import numpy as np
from matplotlib import pyplot as plt
import matplotlib.colors
from matplotlib.animation import FuncAnimation

MAP_NAME = "11_by_11_maze.json"

# Read in watchman_debug.csv
data = []
with open('../build/watchman_debug.csv', 'r') as file:
    next(file)  # Skip header line
    for line in file:
        parts = line.strip().split(',')
        node_id = int(parts[0])
        parent_id = int(parts[1])
        num_agents = int(parts[2])
        cost = int(parts[3])
        heuristic = int(parts[4])
        f_value = int(parts[5])
        num_seen = int(parts[6])
        poses = [(int(parts[7 + i*2]), int(parts[8 + i*2])) for i in range(num_agents)]
        seen_bitset = parts[7 + num_agents*2]
        data.append((node_id, parent_id, num_agents, poses, cost, heuristic, f_value, num_seen, seen_bitset))

print("Data length:", len(data))

# Backtrack the solution path
id_to_data = {entry[0]: entry for entry in data}

solution_data = []
curr = data[-1]
while curr[0] != 0 and curr[0] != 1:
    solution_data.insert(0, curr)
    curr = id_to_data[curr[1]]


with open("../configs/" + MAP_NAME) as f:
    config = json.load(f)

agents = config["agents"]
map = config["map"]

fig, ax = plt.subplots()

colors = ["white","green","gray","red","yellow","orange","black"]
# colors = ["white","green","gray","red","yellow","orange","pink","black"]
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
    df = solution_data[frame_num]
    node_id, parent_id, num_agents, poses, cost, heuristic, f_value, num_seen, seen_bitset = df
    arr = [r.copy() for r in map_copy]
    for i in range(len(seen_bitset)):
        row = i // len(map[0])
        col = i % len(map[0])
        if arr[row][col] == 0:  # Only mark non obstacle cells
            arr[row][col] = int(seen_bitset[i]) / (len(colors) - 1)

    for (x, y) in poses:
        arr[y][x] = 1 / (len(colors) - 1)

    im.set_array(arr)

    text.set_text(f"Expansion #: {frame_num}, Node ID: {node_id}, Pos: ({x},{y}), Cost: {cost}, Heuristic: {heuristic}, F: {f_value}, Seen: {num_seen}")

    return [im, text] # Return a list of artists that were modified

name = sys.argv[1]
fps = int(sys.argv[2])

print("Creating animation...")
print("Solution length: ", len(solution_data))
num_frames = len(solution_data)
# num_frames = 100
interval_ms = 1000 // fps # 50 milliseconds between frames
anim = FuncAnimation(fig, animate_func, frames=num_frames, interval=interval_ms, blit=True)
anim.save(f'{name}.mp4', writer='ffmpeg', fps=fps)
