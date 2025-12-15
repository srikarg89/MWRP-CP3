import sys
import numpy as np
from matplotlib import pyplot as plt

plt.rcParams["axes.labelsize"] = 22       # x/y labels
plt.rcParams["xtick.labelsize"] = 18      # x ticks
plt.rcParams["ytick.labelsize"] = 18      # y ticks
plt.rcParams["legend.fontsize"] = 18      # legend
plt.rcParams["figure.titlesize"] = 18     # figure title (optional)


weights = [1.2, 1.5, 2, 3, 5, 10]
weights = [str(w) for w in weights]
x_pos = np.arange(len(weights))

mxwastar_times = [199252, 28396, 15120, 8330, 8290, 8335]
mxwastar_sol_quality = [68, 79, 73, 111, 118, 182]
mxwastar_times = [t / 1000 for t in mxwastar_times]

soc_times = [371677, 24213, 17537, 16054, 16055, 16490]
soc_sol_quality = [70, 86, 114, 163, 191, 191]
soc_times = [t / 1000 for t in soc_times]

moc_times = [142710, 62169, 72196, 72957, 72880, 75585]
moc_sol_quality = [69, 86, 114, 114, 114, 114]
moc_times = [t / 1000 for t in moc_times]

fig, ax = plt.subplots()

ax.plot(weights, mxwastar_times, label='MxWA*', marker='o', color='red', lw=2)
ax.plot(weights, soc_times, label='FS (SORC)', marker='o', color='purple', lw=2)
ax.plot(weights, moc_times, label='FS (MORC)', marker='o', color='gray', lw=2)
ax.set_ylabel('Runtime (s)')
ax.set_yscale('log')
ax.set_ybound(lower=5, upper=1000)
ax.set_title('32x32 Maze Map, 6 Agents', fontsize=22)
ax.set_xlabel('Weight')

ax2 = ax.twinx()
bar_width = 0.25
ax2.bar(x_pos - bar_width, mxwastar_sol_quality, width=bar_width, alpha=0.4, color='red')
ax2.bar(x_pos, soc_sol_quality, width=bar_width, alpha=0.4, color='purple')
ax2.bar(x_pos + bar_width, moc_sol_quality, width=bar_width, alpha=0.4, color='gray')
ax2.set_ylabel('Solution Cost')

# ax2.legend(loc='lower center', bbox_to_anchor=(0.5, -0.01), ncol=3, frameon=True, edgecolor='black')
fig.legend(loc='lower center', bbox_to_anchor=(0.5, -0.16), ncol=3, frameon=True, edgecolor='black')


# plt.legend()
# plt.show()
plt.savefig('subopt_graph.png', bbox_inches='tight', pad_inches=0.05, dpi=500)
