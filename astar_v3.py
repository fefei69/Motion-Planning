
'''
Improve Collision checking by using R-trees
'''
from pqdict import pqdict
import numpy as np
import math
from tqdm import tqdm
from utils import *
from rtree import index
import uuid
class AStarNode(object):
  def __init__(self, pqkey:tuple, position:np.ndarray, parent_position:tuple=None):
    self.pqkey = pqkey
    self.position = position
    self.g = math.inf
    self.fval = self.g
    # self.h = hval
    self.parent_position = parent_position
    # self.parent_action = None
    # self.closed = False
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

    self.resolution = 0.1
    p = index.Property()
    p.dimension = 3
    self.obs = index.Index(self.obstacle_generator(blocks[:,0:6]), interleaved=True, properties=p)
  
  @staticmethod
  def obstacle_generator(obstacles):
    """
    Add obstacles to r-tree
    :param obstacles: list of obstacles
    """
    for obstacle in obstacles:
      yield (uuid.uuid4().int, obstacle, obstacle)
  
  @staticmethod
  def steer(start, goal, d):
    """
    Return a point in the direction of the goal, that is distance away from start
    :param start: start location
    :param goal: goal location
    :param d: distance away from start
    :return: point in the direction of the goal, distance away from start
    """
    start, end = np.array(start), np.array(goal)
    v = end - start
    u = v / (np.sqrt(np.sum(v ** 2)))
    d = min(d, np.linalg.norm(goal - start))
    steered_point = start + u * d
    return tuple(steered_point)
  
  def es_points_along_line(self, start, end, r):
    """
    Equally-spaced points along a line defined by start, end, with resolution r
    :param start: starting point
    :param end: ending point
    :param r: maximum distance between points
    :return: yields points along line from start to end, separated by distance r
    """
    d = np.linalg.norm(np.array(start) - np.array(end))
    n_points = int(np.ceil(d / r))
    if n_points > 1:
        step = d / (n_points - 1)
        for i in range(n_points):
            next_point = self.steer(start, end, i * step)
            yield next_point

  def obstacle_free(self, x):
    """
    Check if a location resides inside of an obstacle
    :param x: location to check
    :return: True if not inside an obstacle, False otherwise
    """
    return self.obs.count(x) == 0
  
  def collision_checking_v2(self, start, end):
    """
    Check if a line segment intersects an obstacle
    :param start: starting point of line
    :param end: ending point of line
    :param r: resolution of points to sample along edge when checking for collisions
    :return: True if line segment does not intersect an obstacle, False otherwise
    """
    r = self.resolution/2
    points = self.es_points_along_line(start, end, r)
    coll_free = all(map(self.obstacle_free, points))
    return coll_free
  

class AStar(object):
  def __init__(self, environment:Environment, ax, map_resolution, animation=False ,planning_horizon = 2000, num_dirs = 26):
    self.environment = environment
    self.motion = self.motion_model()
    self.horizon = planning_horizon
    self.num_dirs = num_dirs
    self.goal_pos = environment.goal_pos
    self.ax = ax
    self.animation = animation
    self.map_resolution = map_resolution
  def plan(self, epsilon = 1):
    start_pos = self.environment.start_pos
    # Use position as the key, Node objects as the value
    NODES = pqdict()
    OPEN = pqdict()
    CLOSED = pqdict()
    motion = self.motion
    num_dirs = self.num_dirs
    horizon = self.horizon
    boundary = self.environment.boundary  
    # Initial node
    curr = AStarNode(self.meter2grid(start_pos), start_pos, None)
    curr.fval = 0 + self.calc_heuristic(start_pos)
    curr.g = 0
    OPEN[self.meter2grid(start_pos)] = curr.fval
    NODES[self.meter2grid(start_pos)] = curr
    pbar = tqdm(total = horizon)
    # A* planning algorithm
    while True:
      # find minimum f value in OPEN list
      curr_key:tuple = OPEN.pop()
      curr = NODES[curr_key]
      CLOSED[curr_key] = curr
      if pbar.n % 500 == 0:
        print(f"{np.linalg.norm(curr.position-self.goal_pos)} away from goal.")
      
      if self.animation:
        self.update_plot(curr.position, OPEN, CLOSED, NODES) 
      # Expand nodes from current node using motion model
      for k in range(num_dirs):
        next_position = curr.position + motion[:,k] # in meters
        next_position_key = self.meter2grid(next_position)
        motion_cost = np.linalg.norm(motion[:,k])
        # boundary check
        if self.check_out_of_boundary(next_position)==True:
          continue
        
        # collision check
        if self.environment.collision_checking_v2(curr.position, next_position)==False:
          continue

        New_Node = AStarNode(next_position_key, 
                             next_position, 
                             tuple(curr.position))
        New_Node.g = curr.g + motion_cost
        # reach goal condition
        if np.linalg.norm(next_position - self.goal_pos) <= 0.6:
          print("Goal reached!")
          goal_node = AStarNode(self.meter2grid(self.goal_pos), 
                                    self.goal_pos, 
                                    tuple(next_position))
          if self.meter2grid(self.goal_pos)!=next_position_key:
            CLOSED[next_position_key] = New_Node
            CLOSED[self.meter2grid(self.goal_pos)] = goal_node
          else:
            goal_node.parent_position = tuple(curr.position)
            CLOSED[self.meter2grid(self.goal_pos)] = goal_node

          pbar.close()
          return CLOSED, OPEN, NODES

        # j should not be in CLOSED
        if next_position_key in CLOSED:
          continue

        # j not in OPEN, add j to OPEN
        if next_position_key not in OPEN:
          NODES[next_position_key] = New_Node
          # update g value
          OPEN[next_position_key] = New_Node.g + self.calc_heuristic(next_position)
        elif NODES[next_position_key].g > curr.g + motion_cost:
          NODES[next_position_key] = New_Node
          # found a better path
          OPEN[next_position_key] = New_Node.g + self.calc_heuristic(next_position) #update f value
      pbar.update(1)
    # pbar.close()
    # return CLOSED, OPEN
    
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
    r = self.map_resolution*np.ones(3)
    x = position
    return tuple(np.floor((x-m)/r).astype(int))
  def check_out_of_boundary(self, next_position):
    boundary = self.environment.boundary
    if next_position[0]>=boundary[0][0] and \
       next_position[0]<=boundary[0][3] and \
       next_position[1]>=boundary[0][1] and \
       next_position[1]<=boundary[0][4] and \
       next_position[2]>=boundary[0][2] and \
       next_position[2]<=boundary[0][5]:
      return False
    return True
  def update_plot(self, current_pos, open, closed, graph):
    ax:Axes3D = self.ax
    ax.scatter(current_pos[0], current_pos[1], current_pos[2], s=5, color='blue', label='Current',marker='.')
    if open:
      open_nodes = [graph[node].position for node in open.keys()]
      open_nodes = np.array(open_nodes)
      ax.scatter(open_nodes[:,0], open_nodes[:,1], open_nodes[:,2], s=2, color='blue', label='Open', marker='.')
    if closed:
      closed_nodes = [node.position for node in closed.values()]
      closed_nodes = np.array(closed_nodes)
      ax.scatter(closed_nodes[:,0], closed_nodes[:,1], closed_nodes[:,2], s=5, color='grey', label='Closed', marker='.')
    plt.pause(0.0001)


def main(start, goal, mapfile, test_case):
  boundary, blocks = load_map(mapfile)
  ANIMATION = True
  if test_case == 'maze':
    MAP_RESOLUTION = 0.3
    HORIZON = 15000
    ALPHA = 0.05
  elif test_case == 'single_cube':
    MAP_RESOLUTION = 0.1
    HORIZON = 2000
    ALPHA = 0.4
  else:
    MAP_RESOLUTION = 0.3
    HORIZON = 15000
    ALPHA = 0.2
  # TODO: change the boundary and blocks to match the map
  env = Environment(start, 
                    goal, 
                    boundary=boundary, 
                    blocks=blocks)
  
  fig, ax, hb, hs, hg = draw_map(boundary, blocks, start, goal)
  t0 = tic()
  AStarPlanner = AStar(env, ax=ax, map_resolution=MAP_RESOLUTION ,animation=ANIMATION, planning_horizon = HORIZON, num_dirs = 26)
  closed, open, graph = AStarPlanner.plan()
  toc(t0,"Planning")

  open_nodes = [graph[node].position for node in open.keys()]
  open_nodes = np.array(open_nodes)
  ax.scatter(open_nodes[:,0], open_nodes[:,1], open_nodes[:,2], s=1, marker='s', color='grey',alpha=ALPHA)
  closed_nodes = [node.position for node in closed.values()]
  closed_nodes = np.array(closed_nodes)
  ax.scatter(closed_nodes[:,0], closed_nodes[:,1], closed_nodes[:,2], marker='.', color='orange',alpha=ALPHA)
  import pdb; pdb.set_trace()

  path = [goal]
  last_key = meter2grid(boundary, goal,resolution=MAP_RESOLUTION)
#   prev_pos = closed[last_key].parent_position
#   print(closed[last_key].parent_position)
  # TODO: Make the fowllowing path retrival a function
  while True:
    # last_node:AStarNode = closed[last_key]
    last_node:AStarNode = closed.pop(last_key)
    prev_pos = last_node.parent_position
    if prev_pos is None:
      break
    prev_node = closed[meter2grid(boundary, prev_pos, resolution=MAP_RESOLUTION)]
    path.append(prev_node.position)
    last_key = prev_node.pqkey
    print(f"last_key: {last_key}")
  # path = np.array(list(closed.keys()))
  collision_checking(blocks, path, generate_planes(blocks), ax, verbose=True)
  path = np.array(path)
  ax.plot(path[:,0],path[:,1],path[:,2],'r-')
  print(f"Path Length: {np.sum(np.sqrt(np.sum(np.diff(path,axis=0)**2,axis=1)))}")
  import pdb; pdb.set_trace()

if __name__ == '__main__':  
#   start, goal, mapfile, test_case = test_single_cube() # 0.8s Path length: 8.24 (resolution 0.1), 8.47(resolution 0.3)
#   start, goal, mapfile, test_case = test_maze() # 03:27 Path length: 75.0454 resolution 0.3;
  start, goal, mapfile, test_case = test_flappy_bird() # 04:37 path length: 26.25 (resolution 0.3)
#   start, goal, mapfile, test_case = test_monza() # 02:18 Path length: 76.38 (resolution 0.3)
  # start, goal, mapfile, test_case = test_window() # 06:51 Path length: 26.92478 (resolution 0.3)
  # start, goal, mapfile, test_case = test_tower() # 11:57 Path length: 26.4953 (resolution 0.3)
#   start, goal, mapfile, test_case = test_room() #01:15 Path length: 11.39 (resolution 0.3)
  main(start, goal, mapfile, test_case)

  # closed[meter2grid(boundary, tuple(goal))]
  # np.linalg.norm(closed[meter2grid(boundary, tuple(goal))].parent_node.position-goal)