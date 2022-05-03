import numpy as np
from RRT_Utils import RRTStar, Map
np.random.seed(7)

#   xmin, ymin, zmin, xmax, ymax, zmax
obstacles = [[0, 15, 0, 20, 35, 60],
            [25, 55, 0, 45, 75, 100],
            [80, 80, 0, 100, 100, 100],
            [55, 0, 0, 70, 15, 30],
            [80, 20, 0, 100, 45, 90],
            [55, 80, 0, 75, 100, 80],
            [55, 55, 0, 75, 70, 80],
            [0, 80, 0, 20, 100, 80],
            [35, 15, 0, 50, 30, 60]]

bounds = np.array([0,100])
mapobs = Map(obstacles)
start = np.array([100,0,10])
goal = np.array([30,90,90])

rrt_star = RRTStar(start = start, goal = goal,
              Map = mapobs, max_iter = 500,
              goal_sample_rate = 0.1)

waypoints, min_cost = rrt_star.plan()
mapobs.draw_scene(start, goal, path = waypoints)