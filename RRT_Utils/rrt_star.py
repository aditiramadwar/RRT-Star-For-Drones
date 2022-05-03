from .rrt import *
from math import ceil

class RRTStar(RRT):

    def __init__(self, start, goal, Map,
                 max_extend_length = 10.0,
                 goal_sample_rate = 0.05,
                 max_iter = 200 ):
        super().__init__(start, goal, Map, max_extend_length, goal_sample_rate, max_iter)
        self.final_nodes = []

    def plan(self):
        """Plans the path from start to goal while avoiding obstacles"""
        self.start.cost = 0
        self.tree.add(self.start)
        for i in range(self.max_iter):
            rnd = self.get_random_node()
            nearest_node = self.tree.nearest(rnd)
            new_node = self.steer(nearest_node, rnd)
            if not self.map.collision(nearest_node.p,new_node.p):
              self.add_children(new_node)
        if not self.goal.parent: path = None
        else: path = self.final_path()
        return path, self.goal.cost

    def add_children(self,new_node):
        near_nodes = self.near_nodes(new_node)
        self.get_parent(new_node,near_nodes)
        self.tree.add(new_node)
        self.rewire(new_node,near_nodes)
        if self.dist(new_node,self.goal) <= self.max_extend_length:
          if not self.map.collision(self.goal.p,new_node.p):
            self.final_nodes.append(new_node)
        self.get_parent(self.goal,self.final_nodes)

    def get_parent(self, node, parents):
        for parent in parents:
          if not self.map.collision(node.p,parent.p):
            cost = self.new_cost(parent, node)
            if cost < node.cost:
              node.parent = parent
              node.cost = cost

    def rewire(self, new_node, near_nodes):
        for node in near_nodes:
          self.get_parent(node,[new_node])
        self.update_cost(new_node)

    def near_nodes(self, node):
        nnode = self.tree.len + 1
        r = ceil(5.5*np.log(nnode))
        return self.tree.k_nearest(node,r)

    def new_cost(self, from_node, to_node):
        return from_node.cost + self.dist(from_node, to_node)

    def update_cost(self, parent_node):
        for node in self.tree.all():
            if node.parent == parent_node:
                node.cost = self.new_cost(parent_node, node)
                self.update_cost(node)

