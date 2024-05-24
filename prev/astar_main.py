from astar_planner import *
def main(start, goal, mapfile, test_case):
  boundary, blocks = load_map(mapfile)
  ANIMATION = False
  draw_path = True
  if test_case == 'maze':
    MAP_RESOLUTION = 0.3
    HORIZON = 15000
    ALPHA = 0.05
  elif test_case == 'single_cube':
    MAP_RESOLUTION = 0.3
    HORIZON = 2000
    ALPHA = 0.4
  else:
    MAP_RESOLUTION = 0.3
    HORIZON = 15000
    ALPHA = 0.2
  if ANIMATION:
    draw_path = False
    
  env = Environment(start, 
                    goal, 
                    boundary=boundary, 
                    blocks=blocks)
  
  fig, ax, hb, hs, hg = draw_map(boundary, blocks, start, goal)
  t0 = tic()
  AStarPlanner = AStar(env, fig=fig ,ax=ax, map_resolution=MAP_RESOLUTION ,animation=ANIMATION, planning_horizon = HORIZON, num_dirs = 26)
  closed, open, graph = AStarPlanner.plan(epsilon=1.5)
  print("Open list size: ", len(open), "Closed list size: ", len(closed))
  toc(t0,"Planning")
  draw_nodes(graph, open, closed, ax, ALPHA)
  find_path(closed, blocks, boundary, ax, MAP_RESOLUTION, goal, draw_path=draw_path)

  plt.show(block=True)
if __name__ == '__main__':  
  # start, goal, mapfile, test_case = test_single_cube() 
  # start, goal, mapfile, test_case = test_maze() 
  # start, goal, mapfile, test_case = test_flappy_bird() 
  # start, goal, mapfile, test_case = test_monza() 
  start, goal, mapfile, test_case = test_window() 
  # start, goal, mapfile, test_case = test_tower() 
  # start, goal, mapfile, test_case = test_room() 
  main(start, goal, mapfile, test_case)