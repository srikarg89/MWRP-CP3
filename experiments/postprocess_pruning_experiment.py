file = open("pruning_experiment.csv", "r")
lines = file.readlines()
file.close()

lines = [line.strip().split(",") for line in lines]

data = {}
free_cells = {}
for line in lines[1:]:
    map = line[0].strip()
    method = line[1].strip()
    agents = int(line[2].strip())
    cells_left = int(line[3].strip())
    free = int(line[4].strip())
    time = float(line[5].strip())
    free_cells[map] = free

    if map not in data:
        data[map] = {}
    if method not in data[map]:
        data[map][method] = {}
    if agents not in data[map][method]:
        data[map][method][agents] = []
    data[map][method][agents].append((cells_left, time))

for map in sorted(data.keys()):
    print("Map:", map)
    for method in sorted(data[map].keys()):
        for agents in sorted(data[map][method].keys()):
            entries = data[map][method][agents]
            total_cells_left = sum(entry[0] for entry in entries)
            total_time = sum(entry[1] for entry in entries)
            count = len(entries)
            avg_cells_left = total_cells_left / count
            avg_cells_left_pct = (avg_cells_left / free_cells[map]) * 100
            avg_time = total_time / count
            print(f"\tMethod: {method}, Agents: {agents}, Avg cells left: {avg_cells_left:.2f}, Avg cells left %: {avg_cells_left_pct:.2f}, Avg time: {avg_time:.2f}")


print(free_cells)

"""
Number of nominal free cells on map ../maps/mc-forest.map: 1498
Number of nominal free cells on map ../maps/den101d.map: 1360
Number of nominal free cells on map ../maps/huge/ht_chantry.map: 7461

Average free cells in starting config on map ../maps/mc-forest.map with one agent: 1384.66
Average free cells in starting config on map ../maps/den101d.map with one agent: 1096.60
Average free cells in starting config on map ../maps/huge/ht_chantry.map with one agent: 6524.04

Average free cells in starting config on map ../maps/huge/ht_chantry.map with 2 agents: 5865.20
Average free cells in starting config on map ../maps/huge/ht_chantry.map with 3 agents: 5408.86
Average free cells in starting config on map ../maps/huge/ht_chantry.map with 4 agents: 4801.04
Average free cells in starting config on map ../maps/huge/ht_chantry.map with 5 agents: 4381.36
Average free cells in starting config on map ../maps/huge/ht_chantry.map with 6 agents: 4059.72
Average free cells in starting config on map ../maps/huge/ht_chantry.map with 7 agents: 3462.16
Average free cells in starting config on map ../maps/huge/ht_chantry.map with 8 agents: 3170.60

Map: ../maps/mc-forest.map
	Method: CELL, Agents: 1, Avg cells left: 1126.63, Avg cells left %: 75.21, Avg time: 23.79
	Method: CELL_THEN_PATH, Agents: 1, Avg cells left: 102.40, Avg cells left %: 6.84, Avg time: 96.52
	Method: PATH, Agents: 1, Avg cells left: 102.40, Avg cells left %: 6.84, Avg time: 105.64
Map: ../maps/den101d.map
	Method: CELL, Agents: 1, Avg cells left: 526.90, Avg cells left %: 38.74, Avg time: 35.46
	Method: CELL_THEN_PATH, Agents: 1, Avg cells left: 20.46, Avg cells left %: 1.50, Avg time: 64.46
	Method: PATH, Agents: 1, Avg cells left: 20.46, Avg cells left %: 1.50, Avg time: 88.30
Map: ../maps/huge/ht_chantry.map
	Method: CELL, Agents: 1, Avg cells left: 3942.52, Avg cells left %: 52.84, Avg time: 937.52
	Method: CELL_THEN_PATH, Agents: 1, Avg cells left: 179.80, Avg cells left %: 2.41, Avg time: 5323.74
	Method: CELL_THEN_PATH, Agents: 2, Avg cells left: 216.36, Avg cells left %: 2.90, Avg time: 5272.44
	Method: CELL_THEN_PATH, Agents: 3, Avg cells left: 237.16, Avg cells left %: 3.18, Avg time: 5252.37
	Method: CELL_THEN_PATH, Agents: 4, Avg cells left: 253.00, Avg cells left %: 3.39, Avg time: 5238.78
	Method: CELL_THEN_PATH, Agents: 5, Avg cells left: 247.62, Avg cells left %: 3.32, Avg time: 5250.03
	Method: CELL_THEN_PATH, Agents: 6, Avg cells left: 264.38, Avg cells left %: 3.54, Avg time: 5232.46
	Method: CELL_THEN_PATH, Agents: 7, Avg cells left: 295.00, Avg cells left %: 3.95, Avg time: 5220.12
	Method: CELL_THEN_PATH, Agents: 8, Avg cells left: 245.34, Avg cells left %: 3.29, Avg time: 5234.72
	Method: PATH, Agents: 1, Avg cells left: 179.80, Avg cells left %: 2.41, Avg time: 12317.73
	Method: PATH, Agents: 2, Avg cells left: 216.36, Avg cells left %: 2.90, Avg time: 12224.21
	Method: PATH, Agents: 3, Avg cells left: 237.16, Avg cells left %: 3.18, Avg time: 12206.66
	Method: PATH, Agents: 4, Avg cells left: 253.00, Avg cells left %: 3.39, Avg time: 12123.91
	Method: PATH, Agents: 5, Avg cells left: 247.62, Avg cells left %: 3.32, Avg time: 12139.23
	Method: PATH, Agents: 6, Avg cells left: 264.38, Avg cells left %: 3.54, Avg time: 12156.26
	Method: PATH, Agents: 7, Avg cells left: 295.00, Avg cells left %: 3.95, Avg time: 12151.46
	Method: PATH, Agents: 8, Avg cells left: 245.34, Avg cells left %: 3.29, Avg time: 12112.40
"""