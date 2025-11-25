import os
import numpy as np
from matplotlib import pyplot as plt

# MC Forest 3 robots
# filenames = [
#     "anytime_data/mc3_200_mwastar_w2.csv",
#     "anytime_data/mc3_200_mwastar_w3.csv",
#     # "anytime_data/solutions_found_mc-forest.map_3_200.000000_mwastar_5.000000.csv",
#     # "anytime_data/solutions_found_mc-forest.map_3_200.000000_mwastar_10.000000.csv",
#     "anytime_data/mc3_200_soc_w2.csv",
#     "anytime_data/mc3_200_soc_w3.csv",
# ]

# alg_names = [
#     "AMWAstar (w=2)",
#     "AMWAstar (w=3)",
#     # "AMWAstar (w=5)",
#     # "AMWAstar (w=10)",
#     "AFS (SORC) (w=2)",
#     "AFS (SORC) (w=3)",
# ]

# HT Chantry Duo
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
    # "AMWAstar (w=2)",
    "AMWAstar (w=3)",
    "AMWAstar (w=5)",
    # "AMWAstar (w=10)",
    # "AFS (SORC) (w=2)",
    "AFS (SORC) (w=3)",
    "AFS (SORC) (w=5)",
    # "AFS (SORC) (w=10)",
]


# MC Forest Duo
filenames = [
    "anytime_data/solutions_found_mc-forest.map_2_100.000000_mwastar_2.000000.csv",
    "anytime_data/solutions_found_mc-forest.map_2_100.000000_mwastar_3.000000.csv",
    "anytime_data/solutions_found_mc-forest.map_2_100.000000_mwastar_5.000000.csv",
    # "anytime_data/solutions_found_mc-forest.map_2_100.000000_mwastar_10.000000.csv",
    "anytime_data/solutions_found_mc-forest.map_2_100.000000_soc_2.000000.csv",
    "anytime_data/solutions_found_mc-forest.map_2_100.000000_soc_3.000000.csv",
    "anytime_data/solutions_found_mc-forest.map_2_100.000000_soc_5.000000.csv",
    # "anytime_data/solutions_found_mc-forest.map_2_100.000000_soc_10.000000.csv",
]

alg_names = [
    "AMWAstar (w=2)",
    "AMWAstar (w=3)",
    "AMWAstar (w=5)",
    # "AMWAstar (w=10)",
    "AFS (SORC) (w=2)",
    "AFS (SORC) (w=3)",
    "AFS (SORC) (w=5)",
    # "AFS (SORC) (w=10)",
]


datas = []
for filename in filenames:
    data = np.loadtxt(filename, delimiter=",")
    data = np.vstack([data, [1000, data[-1,1]]])
    datas.append(data)

for alg_name, data in zip(alg_names, datas):
    # plt.scatter(data[:,0], data[:,1], label=alg_name, s=10)
    plt.plot(data[:,0], data[:,1], label=alg_name)

plt.legend()

# plt.xlim(0, 180)
# plt.ylim(150, 300)

# plt.xlim(0, 180)
# plt.ylim(600, 630)


plt.show()
