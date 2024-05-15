
# priority queue for OPEN list
from pqdict import pqdict
import numpy as np
import math
from utils import *

class AStarNode(object):
  def __init__(self, pqkey:tuple, position:np.ndarray, parent_node:tuple=None):
    self.pqkey = pqkey
    self.position = position
    self.g = math.inf
    # self.fval = fval
    # self.h = hval
    self.parent_node = parent_node
    self.parent_action = None
    self.closed = False
  def __lt__(self, other):
    return self.g < other.g     
  def __str__(self) -> str:
    return f" grid:{self.pqkey}, position: {self.position}, g value: {self.g}, parent_node: {self.parent_node}"

class Environment(object):
  def __init__(self, start_pos, goal_pos, boundary, blocks):
    self.start_pos = start_pos
    self.goal_pos = goal_pos
    self.boundary = boundary
    self.blocks = blocks
    
  def calc_heuristic(self, position):
    w = 1.2  # weight of heuristic
    d = w * np.linalg.norm(position - self.goal_pos)
    return d
  def collision_check(self, path):
    _, is_collision= collision_checking(self.blocks, path, ax=None, verbose=False)
    return is_collision
  

class AStar(object):
  def __init__(self, environment:Environment, planning_horizon = 2000, num_dirs = 26):
    self.environment = environment
    self.motion = self.motion_model()
    self.horizon = planning_horizon
    self.num_dirs = num_dirs
    self.goal_pos = environment.goal_pos

  def plan(self, epsilon = 1):
    start_pos = self.environment.start_pos
    # Use position as the key, Node objects as the value
    OPEN = pqdict()
    CLOSED = pqdict()
    motion = self.motion
    num_dirs = self.num_dirs
    horizon = self.horizon
    is_goal_reached = False
    # Initial node
    curr = AStarNode(self.meter2grid(start_pos), start_pos, None)
    curr.g = 0 + self.calc_heuristic(start_pos)
    OPEN[self.meter2grid(start_pos)] = curr
    # TODO: Implement A* here
    # for _ in range(horizon):
    while is_goal_reached == False:
      # find minimum f value in OPEN list
      curr_key = min(OPEN, key=lambda o: OPEN[o].g + self.calc_heuristic(OPEN[o].position))
      curr:AStarNode = OPEN.pop(curr_key)
      CLOSED[curr_key] = curr
      CLOSED[curr_key].closed = True
      # Expand nodes from current node using motion model
      for k in range(num_dirs):
        next_position = curr.position + motion[:,k] # in meters
        next_position_key = self.meter2grid(next_position)
        motion_cost = np.linalg.norm(motion[:,k])
        New_Node = AStarNode(next_position_key, 
                             next_position, 
                             curr)
        # reach goal condition
        if np.linalg.norm(next_position - self.goal_pos) < 1:
          print("Goal reached!")
          is_goal_reached = True
          goal_node = AStarNode(self.meter2grid(self.goal_pos), 
                                self.goal_pos, 
                                New_Node)
          CLOSED[next_position_key] = New_Node
          CLOSED[self.meter2grid(self.goal_pos)] = goal_node

        # j not in CLOSED
        if next_position_key in CLOSED:
          continue

        # collision check
        if self.environment.collision_check([curr.position, next_position])==True:
          continue

        # j not in OPEN, add j to OPEN
        if next_position_key not in OPEN:
          # update g value
          New_Node.g = curr.g + motion_cost
          OPEN[next_position_key] = New_Node
        elif OPEN[next_position_key].g > curr.g + motion_cost:
          # found a better path
          OPEN[next_position_key] = New_Node 
          OPEN[next_position_key].g = curr.g + motion_cost

    return CLOSED, OPEN
    
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
  
  def meter2grid(self, position):
    '''
    boundary = [['xmin', 'ymin', 'zmin', 'xmax', 'ymax', 'zmax','r','g','b']]
    x: position in meters
    m: minimum value of the map
    r: resolution of the map
    return: position in grid (cell)
    '''
    m = self.environment.boundary[0,:3]
    r = 0.1*np.ones(3)
    x = position
    return tuple(np.floor((x-m)/r).astype(int))


def main():
  # start, goal, mapfile = test_single_cube()
  # start, goal, mapfile = test_maze()
  start, goal, mapfile = test_flappy_bird()
  # start, goal, mapfile = test_monza()
  # start, goal, mapfile = test_window()
  # start, goal, mapfile = test_tower()
  # start, goal, mapfile = test_room()
  boundary, blocks = load_map(mapfile)

  # TODO: change the boundary and blocks to match the map
  env = Environment(start, 
                    goal, 
                    boundary=boundary, 
                    blocks=blocks)
  t0 = tic()
  AStarPlanner = AStar(env, planning_horizon = 90, num_dirs = 26)
  closed, open = AStarPlanner.plan()
  toc(t0,"Planning")
  fig, ax, hb, hs, hg = draw_map(boundary, blocks, start, goal)

  open_nodes = [node.position for node in open.values()]
  open_nodes = np.array(open_nodes)
  ax.plot(open_nodes[:,0], open_nodes[:,1], open_nodes[:,2], 'b.')
  closed_nodes = [node.position for node in closed.values()]
  closed_nodes = np.array(closed_nodes)
  ax.plot(closed_nodes[:,0], closed_nodes[:,1], closed_nodes[:,2], 'g.')
  # import pdb; pdb.set_trace()

  path = [goal]
  last_key = meter2grid(boundary, goal)
  prev_node = closed[last_key].parent_node
  print(closed[last_key].parent_node)
  # TODO: Convert meters to cells
  while True:
    last_node:AStarNode = closed.pop(last_key)
    prev_node:AStarNode = last_node.parent_node
    if prev_node is None:
      break
    path.append(prev_node.position)
    last_key = prev_node.pqkey
    print(f"last_key: {last_key}")
  # path = np.array(list(closed.keys()))
  path = np.array(path)
  ax.plot(path[:,0],path[:,1],path[:,2],'r-')
  import pdb; pdb.set_trace()

if __name__ == '__main__':  
  main()

  