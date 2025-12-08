import sys
import random

width = int(sys.argv[1])
height = int(sys.argv[2])
pct_obstacles = int(sys.argv[3])
num_obstacles = (width * height * pct_obstacles) // 100


def gen_map():
    map = [["." for _ in range(width)] for _ in range(height)]
    for _ in range(num_obstacles):
        non_obstacles = []
        for i in range(height):
            for j in range(width):
                if map[i][j] == ".":
                    non_obstacles.append((i, j))

        obstacle_cell = random.choice(non_obstacles)
        map[obstacle_cell[0]][obstacle_cell[1]] = "@"
    return map


def validate_map(map):
    # Find free cell.
    free_cell = None
    for i in range(height):
        for j in range(width):
            if map[i][j] == ".":
                free_cell = (i, j)
                break
        if free_cell is not None:
            break
    
    # BFS to find all reachable free cells.
    visited = set()
    queue = [free_cell]
    while queue:
        cell = queue.pop(0)
        if cell in visited:
            continue
        visited.add(cell)
        x, y = cell
        for dx, dy in [(-1, 0), (1, 0), (0, -1), (0, 1)]:
            nx, ny = x + dx, y + dy
            if 0 <= nx < height and 0 <= ny < width and map[nx][ny] == "." and (nx, ny) not in visited:
                queue.append((nx, ny))
    
    # Check if all free cells are reachable.
    for i in range(height):
        for j in range(width):
            if map[i][j] == "." and (i, j) not in visited:
                return False
    return True


map = gen_map()
while not validate_map(map):
    map = gen_map()


file = open(f"../maps/random-{width}-{height}-{num_obstacles}.map", "w+")
file.write(f"type octile\n")
file.write(f"height {height}\n")
file.write(f"width {width}\n")
file.write(f"map\n")
for row in map:
    file.write("".join(row) + "\n")
file.close()
