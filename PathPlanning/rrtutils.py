from rtree import index
import numpy as np

class Node:
    def __init__(self,coords):
        self.p = np.array(coords)
        self.parent = None
        self.cost = np.inf

    def __len__(self):
        return len(self.p)

    def __getitem__(self, i):
        return self.p[i]

    def __repr__(self):
        return 'Node({}, {})'.format(self.p,self.cost)

#Rtree to store Nodes
class Rtree:
  def __init__(self):
    self.node_list = []
    self.idx = self.get_tree()
    self.len = 0

  @staticmethod
  def get_tree():
    p = index.Property()
    p.dimension = 3
    p.dat_extension = 'data'
    p.idx_extension = 'index'
    return index.Index(properties=p)

  def add(self,new_node):
    self.node_list.append(new_node)
    self.idx.insert(self.len,(*new_node.p,))
    self.len += 1

  def k_nearest(self,node,k):
    near_ids = self.idx.nearest((*node.p,),k)
    for i in near_ids:
      yield self.node_list[i]

  def nearest(self,node):
    near_ids = self.idx.nearest((*node.p,),1)
    id = list(near_ids)[0]
    return self.node_list[id]

  def all(self):
    return self.node_list
