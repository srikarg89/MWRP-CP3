import csv

new_data = open('../build/watchman_debug_new.csv').readlines()[1:]
old_data = open('../build/watchman_debug_old.csv').readlines()[1:]

new_data = [line.split(',') for line in new_data]
old_data = [line.split(',') for line in old_data]

def find_line_in_old(line):
    for old_line in old_data:
        if line[-1] == old_line[-1] and line[-2] == old_line[-2] and line[-3].strip() == old_line[-3].strip():
            return old_line
    return None

lookup = {}
not_found = []
for line in new_data:
    old_line = find_line_in_old(line)
    if old_line is None:
        # print("New line not found in old data:")
        # print(line)
        not_found.append(line)
    else:
        lookup[line[0]] = old_line

    # else:
    #     if line[3] != old_line[3]:
    #         print("Cost mismatch:")
    #         print("\tNew: ", line)
    #         print("\tOld: ", old_line)
    #     if line[4] != old_line[4]:
    #         print("Heuristic mismatch:")
    #         print("\tNew: ", line)
    #         print("\tOld: ", old_line)
    #     if line[5] != old_line[5]:
    #         print("F Value mismatch:")
    #         print("\tNew: ", line)
    #         print("\tOld: ", old_line)




