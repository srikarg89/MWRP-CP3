INPUT_MAP_NAME = "../maps/mc-forest.map"
OUTPUT_MAP_NAME = "../maps/mc-forest-doubled.map"

with open(INPUT_MAP_NAME) as f:
    lines = f.readlines()
    map_lines = lines[4:] # Skip the first four header lines
    map_lines = [line.strip() for line in map_lines if line.strip()]  # Remove any empty lines
    map = [[0 if c == '.' else 1 for c in line.strip()] for line in map_lines]

doubled_map = []
for row in map:
    new_row = []
    for cell in row:
        new_row.append(cell)
        new_row.append(cell)
    doubled_map.append(new_row.copy())
    doubled_map.append(new_row.copy())


height = len(doubled_map)
width = len(doubled_map[0])

with open(OUTPUT_MAP_NAME, 'w') as f:
    f.write(f"type octile\n")
    f.write(f"height {height}\n")
    f.write(f"width {width}\n")
    f.write(f"map\n")
    for row in doubled_map:
        line = ''.join(['.' if cell == 0 else 'T' for cell in row])
        f.write(line + '\n')
