import sys
import random

num = 4
for i in range(num):
    wall_num = [random.randint(0, 2) for _ in range(4)]
    room = [['.', '.', '.', '.'] for _ in range(4)]
    for x in range(4):
        for y in range(4):
            if x == 0 or y == 0 or x == 3 or y == 3:
                room[x][y] = '@'
    
    for w in range(wall_num[0]):
        spot = random.randint(0, 3)
        room[0][spot] = '.'

    for w in range(wall_num[1]):
        spot = random.randint(0, 3)
        room[3][spot] = '.'

    for w in range(wall_num[2]):
        spot = random.randint(0, 3)
        room[spot][0] = '.'

    for w in range(wall_num[3]):
        spot = random.randint(0, 3)
        room[spot][3] = '.'

    print('\n'.join([''.join(row) for row in room]))
    print()
