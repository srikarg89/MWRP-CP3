import os
import numpy as np
from matplotlib import pyplot as plt

THING = 4

# HT Chantry Duo
if THING == 1:
    filenames = [
    #     "anytime_data/solutions_found_ht_chantry.map_2_200.000000_mwastar_2.000000.csv",
        "anytime_data/solutions_found_ht_chantry.map_2_200.000000_mwastar_3.000000.csv",
        "anytime_data/solutions_found_ht_chantry.map_2_200.000000_mwastar_5.000000.csv",
        # "anytime_data/solutions_found_ht_chantry.map_2_200.000000_mwastar_10.000000.csv",
        # "anytime_data/solutions_found_ht_chantry.map_2_200.000000_soc_2.000000.csv",
        "anytime_data/solutions_found_ht_chantry.map_2_200.000000_soc_3.000000.csv",
        "anytime_data/solutions_found_ht_chantry.map_2_200.000000_soc_5.000000.csv",
        # "anytime_data/solutions_found_ht_chantry.map_2_200.000000_soc_10.000000.csv",
    ]

    alg_names = [
        # "AMWA* (w=2)",
        "AMWA* (w=3)",
        "AMWA* (w=5)",
        # "AMWA* (w=10)",
        # "AFS (SORC) (w=2)",
        "AFS (SORC) (w=3)",
        "AFS (SORC) (w=5)",
        # "AFS (SORC) (w=10)",
    ]


# MC Forest Trio
elif THING == 3:
    filenames = [
        "anytime_data/mc3_200_mwastar_w2.csv",
        "anytime_data/mc3_200_mwastar_w3.csv",
        # "anytime_data/solutions_found_mc-forest.map_3_200.000000_mwastar_5.000000.csv",
        # "anytime_data/solutions_found_mc-forest.map_3_200.000000_mwastar_10.000000.csv",
        "anytime_data/mc3_200_soc_w2.csv",
        "anytime_data/mc3_200_soc_w3.csv",
    ]

    alg_names = [
        "AMWA* (w=2)",
        "AMWA* (w=3)",
        # "AMWA* (w=5)",
        # "AMWA* (w=10)",
        "AFS (SORC) (w=2)",
        "AFS (SORC) (w=3)",
    ]

# MC Forest Duo
elif THING == 4:
    filenames = [
        "anytime_data/solutions_found_mc-forest.map_2_300.000000_mwastar_2.000000.csv",
        "anytime_data/solutions_found_mc-forest.map_2_300.000000_mwastar_3.000000.csv",
        # "anytime_data/solutions_found_mc-forest.map_2_300.000000_mwastar_5.000000.csv",
        # "anytime_data/solutions_found_mc-forest.map_2_300.000000_mwastar_10.000000.csv",

        "anytime_data/solutions_found_mc-forest.map_2_300.000000_soc_2.000000.csv",
        "anytime_data/solutions_found_mc-forest.map_2_300.000000_soc_3.000000.csv",
        # "anytime_data/solutions_found_mc-forest.map_2_300.000000_soc_5.000000.csv",
        # "anytime_data/solutions_found_mc-forest.map_2_300.000000_soc_10.000000.csv",

        "anytime_data/solutions_found_mc-forest.map_2_300.000000_moc_2.000000.csv",
        "anytime_data/solutions_found_mc-forest.map_2_300.000000_moc_3.000000.csv",
        # "anytime_data/solutions_found_mc-forest.map_2_300.000000_moc_5.000000.csv",
        # "anytime_data/solutions_found_mc-forest.map_2_300.000000_moc_10.000000.csv",
    ]

    alg_names = [
        "AMWA* (w=2)",
        "AMWA* (w=3)",
        # "AMWA* (w=5)",
        # "AMWA* (w=10)",

        "AFS (SORC) (w=2)",
        "AFS (SORC) (w=3)",
        # "AFS (SORC) (w=5)",
        # "AFS (SORC) (w=10)",

        "AFS (MORC) (w=2)",
        "AFS (MORC) (w=3)",
        # "AFS (MORC) (w=5)",
        # "AFS (MORC) (w=10)",
    ]

datas = []
for filename in filenames:
    data = np.loadtxt(filename, delimiter=",")
    data = np.vstack([data, [1000, data[-1,1]]])
    # datas.append(data)

    data2 = []
    for d in data:
        if len(data2) > 0:
            data2.append([d[0], data2[-1][1]])
        data2.append(d)
    datas.append(np.array(data2))    

plt.rcParams.update({'font.size': 20})

fig, ax = plt.subplots(figsize=(15, 6))

for alg_name, data in zip(alg_names, datas):
    if alg_name == "AFS (SORC) (w=2)" and THING == 4:
        ax.scatter(data[:,0], data[:,1], s=5)
        ax.plot(data[:,0], data[:,1], linewidth=3)
        ax.plot([], [], label="AFS (SORC) (w=2)", linewidth=2)
    else:
        ax.scatter(data[:,0], data[:,1], s=5)
        ax.plot(data[:,0], data[:,1], label=alg_name, linewidth=2)

ax.legend(ncol=2, loc='upper right')

if THING == 1:
    ax.set_xlim(0, 70)
elif THING == 2:
    ax.set_xlim(0, 100)
elif THING == 3:
    ax.set_xlim(0, 140)
elif THING == 4:
    ax.set_xlim(0, 300)

ax.set_xlabel("Time (s)")
ax.set_ylabel("Solution Cost")

# plt.xlim(0, 180)
# plt.ylim(150, 300)

# plt.xlim(0, 180)
# plt.ylim(600, 630)

plt.savefig("anytime_plot.png", bbox_inches='tight', pad_inches=0.25)
# plt.show()
