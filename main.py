from rtree import index
import matplotlib.pyplot as plt
import numpy as np
import mpl_toolkits.mplot3d.axes3d as Axes3D

def cuboid_data(box):
    l = box[3] - box[0]
    w = box[4] - box[1]
    h = box[5] - box[2]
    x = [[0, l, l, 0, 0],
         [0, l, l, 0, 0],
         [0, l, l, 0, 0],
         [0, l, l, 0, 0]]
    y = [[0, 0, w, w, 0],
         [0, 0, w, w, 0],
         [0, 0, 0, 0, 0],
         [w, w, w, w, w]]
    z = [[0, 0, 0, 0, 0],
         [h, h, h, h, h],
         [0, 0, h, h, 0],
         [0, 0, h, h, 0]]
    return box[0] + np.array(x), box[1] + np.array(y), box[2] + np.array(z)

#xmin, ymin, zmin=0, xmax, ymax, zmax
obstacle = [[15, 30, 0, 50, 50, 30],
            [15, 65, 0, 50, 80, 60]]

ax = plt.axes(projection ='3d')
obstacles = np.array(obstacle)
for box in obstacles:
    X, Y, Z = cuboid_data(box)
    ax.plot_surface(X, Y, Z, alpha = 0.3)
plt.show()
