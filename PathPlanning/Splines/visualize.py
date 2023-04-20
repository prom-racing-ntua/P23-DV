import sys
import matplotlib.pyplot as plt
from matplotlib.lines import Line2D

try:
    path = sys.argv[1]
except IndexError:
    path = "./PathPlanning/Splines/build/arc_length_spline_points.txt"

f = open(path, 'r')

X = []
Y = []
for row in f:
    row = row.strip().split()
    X.append(float(row[0]))
    Y.append(float(row[1]))

fig, ax = plt.subplots()
fig.set_size_inches(13,9)
canvas = ax.figure.canvas
ax.axis('equal')
ax.set_ylim([-30,50])
ax.set_xlim([-110,40])

# For visualizing the speed on the curve based on the spline parameter (is pretty slow...)
# line = Line2D(X[:1], Y[:1], color='orange', animated=True)
# ax.add_line(line)
# canvas.draw()

# background = canvas.copy_from_bbox(ax.bbox)
# ax.draw_artist(line)

# for i in range(2, len(X), 1):
#     canvas.restore_region(background)
#     line.set_data(X[:i], Y[:i])
#     ax.draw_artist(line)
#     canvas.blit(ax.bbox)
#     plt.pause(0.000000005)


ax.plot(X, Y, color='orange')
plt.show()