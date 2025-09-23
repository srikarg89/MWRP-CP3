import json
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
        x = int(parts[1])
        y = int(parts[2])
        cost = int(parts[3])
        heuristic = int(parts[4])
        f_value = int(parts[5])
        num_seen = int(parts[6])
        seen_bitset = parts[7]
        data.append((node_id, x, y, cost, heuristic, f_value, num_seen, seen_bitset))

print("Data length:", len(data))

with open("../configs/" + MAP_NAME) as f:
    config = json.load(f)

agents = config["agents"]
map = config["map"]

fig, ax = plt.subplots()

cmap = matplotlib.colors.LinearSegmentedColormap.from_list("", ["white","green","gray","red","yellow","black"])
im = ax.imshow(map, cmap=cmap)

ax.grid(which='major', axis='both', linestyle='-', color='k', linewidth=1.5)
ax.set_xticks(np.arange(-.5, len(map[0]), 1))
ax.set_yticks(np.arange(-.5, len(map), 1))

ax.set_yticklabels([])
ax.set_xticklabels([])

map_copy = [r.copy() for r in map.copy()]

text = plt.text(0, -1, "", fontsize=10)

def animate_func(frame_num):
    print("Animating frame:", frame_num)
    # Generate or load data for the current frame
    # im.set_array(new_data)
    df = data[frame_num]
    node_id, x, y, cost, heuristic, f_value, num_seen, seen_bitset = df
    arr = [r.copy() for r in map_copy]
    for i in range(len(seen_bitset)):
        row = i // len(map[0])
        col = i % len(map[0])
        if arr[row][col] == 0:  # Only mark non obstacle cells
            arr[row][col] = int(seen_bitset[i]) / 5

    arr[y][x] = 1 / 5

    im.set_array(arr)

    text.set_text(f"Node ID: {node_id}, Pos: ({x},{y}), Cost: {cost}, Heuristic: {heuristic}, F: {f_value}, Seen: {num_seen}")

    return [im, text] # Return a list of artists that were modified

print("Creating animation...")
print("Number of frames: ", len(data))
num_frames = len(data)
# num_frames = 100
interval_ms = 50 # 50 milliseconds between frames
anim = FuncAnimation(fig, animate_func, frames=num_frames, interval=interval_ms, blit=True)

anim.save('animation4.mp4', writer='ffmpeg', fps=20)
