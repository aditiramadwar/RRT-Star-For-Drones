from rtree import index
import numpy as np
import matplotlib.pyplot as plt
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

class Map:
  def __init__(self,obstacle_list, path_resolution = 0.5):
    self.obstacles = obstacle_list
    self.idx = self.get_tree(obstacle_list)
    self.len = len(obstacle_list)
    self.path_res = path_resolution

  @staticmethod
  def get_tree(obstacle_list):
    p = index.Property()
    p.dimension = 3
    ls = [(i, (*obj, ), None) for i, obj in enumerate(obstacle_list)]
    return index.Index(ls, properties = p)

  def add(self, obstacle):
    self.idx.insert(self.len,obstacle)
    self.obstacles.append(obstacle)
    self.len += 1

  def collision(self ,start, end):
    dist = np.linalg.norm(start-end)
    n = int(dist/self.path_res)
    points = np.linspace(start,end,n)
    for p in points:
      if self.idx.count((*p,)) != 0 :
          return True
    return False

  def plotobs(self,ax,scale = 1):
    obstacles = scale*np.array(self.obstacles)
    for box in obstacles:
        X, Y, Z = cuboid_data(box)
        ax.plot_surface(X, Y, Z, rstride=1, cstride=1,alpha = 0.2,zorder = 1)


  def draw_scene(self, start, end, path = None, ax = None, graph = False):
    if ax is None:
        fig = plt.figure(figsize=(30,20))
        ax = Axes3D.Axes3D(fig)
        ax.axes.set_xlim3d(left=0, right=100) 
        ax.axes.set_ylim3d(bottom=0, top=100) 
        ax.axes.set_zlim3d(bottom=0, top=100) 
    ax.scatter(float(start[0]), float(start[1]), float(start[2]), "-", s=80, color = (0.2, 0.9, 0.2, 0.9), zorder = 5)
    ax.scatter(float(end[0]), float(end[1]), float(end[2]), "-", s=60, color = (0.9, 0.2, 0.9, 0.9), zorder = 5)
    self.plotobs(ax)
    if path is not None:

        path_x = (path.T)[0]
        path_y = (path.T)[1]
        path_z = (path.T)[2]

        prev_point_x = path_x[0]
        prev_point_y = path_y[0]
        prev_point_z = path_z[0]
        
        for i in range(len(path_x)):
            ax.plot([path_x[i], prev_point_x],[path_y[i], prev_point_y],[path_z[i], prev_point_z], '-', color = (0.9, 0.2, 0.5, 0.8), zorder = 5)
            ax.scatter(path_x[i], path_y[i], path_z[i], "-", color = (0.2, 0.2, 0.9, 0.9), zorder = 5)
            prev_point_x = path_x[i]
            prev_point_y = path_y[i]
            prev_point_z = path_z[i]

            plt.pause(0.1)
    plt.show()