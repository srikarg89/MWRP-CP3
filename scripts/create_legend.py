import matplotlib.pyplot as plt
from matplotlib.lines import Line2D

plt.rcParams["legend.fontsize"] = 12      # legend

# Create dummy handles
ms = 12
mw = 1.5
handles = [
    Line2D([], [], marker='s', markerfacecolor='green', markersize=ms,
           markeredgecolor='black', markeredgewidth=mw,
           linestyle='None', label='Agent'),
    Line2D([], [], marker='s', markerfacecolor='black', markersize=ms,
            markeredgecolor='black', markeredgewidth=mw,
           linestyle='None', label='Obstacle'),
    Line2D([], [], marker='s', markerfacecolor='gray', markersize=ms,
           markeredgecolor='black', markeredgewidth=mw,
           linestyle='None', label='Free Cell (Seen)'),
    Line2D([], [], marker='s', markerfacecolor='white', markersize=ms,
           markeredgecolor='black', markeredgewidth=mw,
           linestyle='None', label='Free Cell (Unseen)'),
    Line2D([], [], marker='s', markerfacecolor='red', markersize=ms,
           markeredgecolor='black', markeredgewidth=mw,
           linestyle='None', label='Pivot'),
    Line2D([], [], marker='s', markerfacecolor='yellow', markersize=ms,
           markeredgecolor='black', markeredgewidth=mw,
           linestyle='None', label='Pivot Watcher'),
]

# handles = [
#     Line2D([], [], marker='s', markersize=14,
#            markerfacecolor='blue', markeredgecolor='black', markeredgewidth=mw,
#            linestyle='None', label='Series A'),
#     Line2D([], [], marker='s', markersize=14,
#            markerfacecolor='red', markeredgecolor='black', markeredgewidth=mw,
#            linestyle='None', label='Series B'),
#     Line2D([], [], marker='s', markersize=14,
#            markerfacecolor='green', markeredgecolor='black', markeredgewidth=mw,
#            linestyle='None', label='Series C'),
# ]

fig = plt.figure()
legend = fig.legend(handles=handles, loc='center', ncol=len(handles), frameon=True, edgecolor='black', columnspacing=0.4)
for line in legend.get_lines():
    line.set_linewidth(3.0)

# legend.set_linewidth(2.5)
fig.tight_layout()

plt.savefig('../images/Fig1 Legend.png', dpi=500, bbox_inches='tight')

# plt.show()
