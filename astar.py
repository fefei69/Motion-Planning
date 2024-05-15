
# priority queue for OPEN list
from pqdict import pqdict
import numpy as np
import math
from utils import *

class AStarNode(object):
  def __init__(self, pqkey:tuple, position:np.ndarray, fval:float, parent_node:tuple=None):
    self.pqkey = pqkey
    self.position = position
    self.g = math.inf
    self.fval = fval
    # self.h = hval
    self.parent_node = parent_node
    self.parent_action = None
    self.closed = False
  def __lt__(self, other):
    return self.g < other.g     
  def __str__(self) -> str:
    return f"position: {self.position}, f value: {self.fval}, parent_node: {self.parent_node}, parent_action: {self.parent_action}"

class Environment(object):
  def __init__(self, start_pos, goal_pos, boundary, blocks):
    self.start_pos = start_pos
    self.goal_pos = goal_pos
    self.boundary = boundary
    self.blocks = blocks
    
  def calc_heuristic(self, position):
    w = 1.0  # weight of heuristic
    d = w * np.linalg.norm(position - self.goal_pos)
    return d
  def collision_check(self, path):
    _, is_collision= collision_checking(self.blocks, path, ax=None, verbose=False)
    return is_collision
  

class AStar(object):
  def __init__(self, environment, planning_horizon = 2000, num_dirs = 26):
    self.environment = environment
    self.motion = self.motion_model()
    self.horizon = planning_horizon
    self.num_dirs = num_dirs
    self.goal_pos = environment.goal_pos

  def plan(self, epsilon = 1):
    start_pos = self.environment.start_pos
    # Initialize the graph and open list
    Graph = {}
    # Use position as the key, Node objects as the value
    OPEN = pqdict()
    CLOSED = pqdict()
    PATH = [start_pos]
    motion = self.motion
    num_dirs = self.num_dirs
    horizon = self.horizon
    # Initial node
    curr = AStarNode(tuple(start_pos), start_pos, self.calc_heuristic(start_pos))
    curr.g = 0
    curr.fval = curr.g + self.calc_heuristic(start_pos)
    OPEN[tuple(start_pos)] = curr
    # TODO: Implement A* here
    for _ in range(horizon):
       # find minimum f value in OPEN list
      curr_key = min(OPEN, key=lambda o: OPEN[o].fval + self.calc_heuristic(OPEN[o].pqkey))
      curr:AStarNode = OPEN.pop(curr_key)
      CLOSED[curr_key] = curr
      CLOSED[curr_key].closed = True
      # Expand nodes from current node using motion model
      for k in range(num_dirs):
        next_position = curr.position + motion[:,k]
        next_position_key = tuple(next_position)
        motion_cost = np.linalg.norm(motion[:,k])
        # TODO: Add reach goal condition
        if next_position_key in CLOSED:
          continue

        if self.environment.collision_check([curr.position, next_position])==True:
          continue

        if next_position_key not in OPEN:
          New_Node = AStarNode(next_position_key, 
                               next_position, 
                               motion_cost+self.calc_heuristic(next_position),
                               curr)
          # update g value
          New_Node.g = curr.g + np.linalg.norm(motion[:,k])
          OPEN[next_position_key] = New_Node
        elif OPEN[next_position_key].fval > curr.g + motion_cost:
          OPEN[next_position_key] = AStarNode(next_position_key, 
                                              next_position, 
                                              motion_cost+self.calc_heuristic(next_position),
                                              curr)
          OPEN[next_position_key].fval = curr.g + motion_cost
          OPEN[next_position_key].parent_node = curr

    print(OPEN)
    return OPEN
    
  @staticmethod
  def motion_model():
    [dX,dY,dZ] = np.meshgrid([-1,0,1],[-1,0,1],[-1,0,1])
    dR = np.vstack((dX.flatten(),dY.flatten(),dZ.flatten()))
    dR = np.delete(dR,13,axis=1)
    dR = dR / np.sqrt(np.sum(dR**2,axis=0)) / 2.0
    return dR
  
  def calc_heuristic(self, position):
    w = 1.0  # weight of heuristic
    d = w * np.linalg.norm(position - self.goal_pos)
    return d


def main():
  mapfile = './maps/single_cube.txt'
  boundary, blocks = load_map(mapfile)
  start = np.array([2.3, 2.3, 1.3])
  goal = np.array([7.0, 7.0, 5.5])

  # TODO: change the boundary and blocks to match the map
  env = Environment(start, 
                    goal, 
                    boundary=boundary, 
                    blocks=blocks)
  
  AStarPlanner = AStar(env, planning_horizon = 15, num_dirs = 26)
  open = AStarPlanner.plan()
  fig, ax, hb, hs, hg = draw_map(boundary, blocks, start, goal)
  open_nodes = np.array(list(open.popkeys()))
  ax.plot(open_nodes[:,0], open_nodes[:,1], open_nodes[:,2], 'g.')
  import pdb; pdb.set_trace()
  print(open_nodes)

if __name__ == '__main__':  
  main()