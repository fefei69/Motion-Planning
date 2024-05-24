
# priority queue for OPEN list
from pqdict import pqdict
import numpy as np
import math
from tqdm import tqdm
from utils import *

class AStarNode(object):
  def __init__(self, pqkey:tuple, position:np.ndarray, parent_position:tuple=None):
    self.pqkey = pqkey
    self.position = position
    self.g = math.inf
    # self.fval = fval
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
    self.block_planes = self.get_block_planes(blocks)
  def collision_check(self, path)->bool:
    _, is_collision= collision_checking_robust(self.blocks, path, self.block_planes, ax=None, verbose=False)
    return is_collision
  @staticmethod
  def get_block_planes(blocks)->dict:
    return generate_planes(blocks)
  

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
    OPEN = pqdict()
    CLOSED = pqdict()
    motion = self.motion
    num_dirs = self.num_dirs
    horizon = self.horizon
    boundary = self.environment.boundary  
    # Initial node
    curr = AStarNode(self.meter2grid(start_pos), start_pos, None)
    curr.g = 0 + self.calc_heuristic(start_pos)
    OPEN[self.meter2grid(start_pos)] = curr
    # TODO: Implement A* here
    pbar = tqdm(total = horizon)
    # for _ in range(horizon):
    while True:
      # find minimum f value in OPEN list
      curr_key = min(OPEN, key=lambda o: OPEN[o].g + self.calc_heuristic(OPEN[o].position))
      # if curr_key ==self.meter2grid(self.goal_pos):
      #   break
      curr:AStarNode = OPEN.pop(curr_key)
      CLOSED[curr_key] = curr
      if pbar.n % 500 == 0:
        print(f"{np.linalg.norm(curr.position-self.goal_pos)} away from goal.")

      # reach goal condition
      # if np.linalg.norm(curr.position - self.goal_pos) <= 1:
      #   print("Goal reached!")
      #   # is_goal_reached = True
      #   goal_node = AStarNode(self.meter2grid(self.goal_pos), 
      #                         self.goal_pos, 
      #                         tuple(curr.position))
      #   CLOSED[curr_key] = curr
      #   CLOSED[self.meter2grid(self.goal_pos)] = goal_node
      #   pbar.close()
      #   return CLOSED, OPEN
      
      if self.animation:
        self.update_plot(curr.position, OPEN, CLOSED) 
      # Expand nodes from current node using motion model
      for k in range(num_dirs):
        next_position = curr.position + motion[:,k] # in meters
        next_position_key = self.meter2grid(next_position)
        motion_cost = np.linalg.norm(motion[:,k])
        New_Node = AStarNode(next_position_key, 
                             next_position, 
                             tuple(curr.position))
        # boundary check
        if self.check_out_of_boundary(next_position)==True:
          continue
        
        # collision check
        if self.environment.collision_check([curr.position, next_position])==True:
          continue

        # reach goal condition
        if np.linalg.norm(next_position - self.goal_pos) <= 1:
          print("Goal reached!")
          # is_goal_reached = True
          goal_node = AStarNode(self.meter2grid(self.goal_pos), 
                                self.goal_pos, 
                                tuple(next_position))
          CLOSED[next_position_key] = New_Node
          CLOSED[self.meter2grid(self.goal_pos)] = goal_node
          pbar.close()
          return CLOSED, OPEN

        # j not in CLOSED
        if next_position_key in CLOSED:
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
  def update_plot(self, current_pos, open, closed):
    ax:Axes3D = self.ax
    ax.scatter(current_pos[0], current_pos[1], current_pos[2], s=5, color='blue', label='Current',marker='.')
    if open:
      open_nodes = [node.position for node in open.values()]
      open_nodes = np.array(open_nodes)
      ax.scatter(open_nodes[:,0], open_nodes[:,1], open_nodes[:,2], s=2, color='blue', label='Open', marker='.')
    if closed:
      closed_nodes = [node.position for node in closed.values()]
      closed_nodes = np.array(closed_nodes)
      ax.scatter(closed_nodes[:,0], closed_nodes[:,1], closed_nodes[:,2], s=5, color='grey', label='Closed', marker='.')
    plt.pause(0.0001)


def main(start, goal, mapfile, test_case):
  boundary, blocks = load_map(mapfile)
  ANIMATION = False
  if test_case == 'maze':
    MAP_RESOLUTION = 0.3
    HORIZON = 15000
    ALPHA = 0.05
  elif test_case == 'single_cube':
    MAP_RESOLUTION = 0.1
    HORIZON = 2000
    ALPHA = 0.2
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
  closed, open = AStarPlanner.plan()
  toc(t0,"Planning")

  open_nodes = [node.position for node in open.values()]
  open_nodes = np.array(open_nodes)
  ax.scatter(open_nodes[:,0], open_nodes[:,1], open_nodes[:,2], s=1, marker='s', color='grey',alpha=ALPHA)
  closed_nodes = [node.position for node in closed.values()]
  closed_nodes = np.array(closed_nodes)
  ax.scatter(closed_nodes[:,0], closed_nodes[:,1], closed_nodes[:,2], marker='.', color='orange',alpha=ALPHA)
  import pdb; pdb.set_trace()

  path = [goal]
  last_key = meter2grid(boundary, goal,resolution=MAP_RESOLUTION)
  prev_pos = closed[last_key].parent_position
  print(closed[last_key].parent_position)
  # TODO: Make the fowllowing path retrival a function
  while True:
    # last_node:AStarNode = closed[last_key]
    last_node:AStarNode = closed.pop(last_key)
    prev_pos:AStarNode = last_node.parent_position
    if prev_pos is None:
      break
    prev_node = closed[meter2grid(boundary, prev_pos, resolution=MAP_RESOLUTION)]
    path.append(prev_node.position)
    last_key = prev_node.pqkey
    print(f"last_key: {last_key}")
  # path = np.array(list(closed.keys()))
  collision_checking_robust(blocks, path, generate_planes(blocks), ax, verbose=True)
  path = np.array(path)
  ax.plot(path[:,0],path[:,1],path[:,2],'r-')
  pathlength = np.linalg.norm(path[1:,:] - path[:-1,:],axis=1).sum()
  print(f"Path length: {pathlength}")
  print(f"Default Path Length: {np.sum(np.sqrt(np.sum(np.diff(path,axis=0)**2,axis=1)))}")
  import pdb; pdb.set_trace()

if __name__ == '__main__':  
  start, goal, mapfile, test_case = test_single_cube() # 0.8s Path length: 8.24 (resolution 0.1), 8.47(resolution 0.3)
  # start, goal, mapfile, test_case = test_maze() #51:24 Path length: 75.4010 resolution 0.3; 21:09 Path length: 80.89711 resolution 0.4
  # start, goal, mapfile, test_case = test_flappy_bird() # 04:37 path length: 26.25 (resolution 0.3)
  # start, goal, mapfile, test_case = test_monza() # 02:18 Path length: 76.38 (resolution 0.3)
  # start, goal, mapfile, test_case = test_window() # 06:51 Path length: 26.92478 (resolution 0.3)
  # start, goal, mapfile, test_case = test_tower() # 11:57 Path length: 26.4953 (resolution 0.3)
  # start, goal, mapfile, test_case = test_room() #01:15 Path length: 11.39 (resolution 0.3)
  main(start, goal, mapfile, test_case)

  # closed[meter2grid(boundary, tuple(goal))]
  # np.linalg.norm(closed[meter2grid(boundary, tuple(goal))].parent_node.position-goal)