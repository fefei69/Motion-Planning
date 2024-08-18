import argparse
from astar_planner import *

def main(start, goal, mapfile, test_case, epsilon=1):
    boundary, blocks = load_map(mapfile)
    ANIMATION = True
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

    env = Environment(start, goal, boundary=boundary, blocks=blocks)

    fig, ax, hb, hs, hg = draw_map(boundary, blocks, start, goal)
    t0 = tic()
    AStarPlanner = AStar(env, fig=fig, ax=ax, map_resolution=MAP_RESOLUTION, animation=ANIMATION, planning_horizon=HORIZON, num_dirs=26)
    closed, open, graph = AStarPlanner.plan(epsilon=epsilon)
    print("Open list size: ", len(open), "Closed list size: ", len(closed))
    toc(t0, "Planning")
    draw_nodes(graph, open, closed, ax, ALPHA)
    find_path(closed, blocks, boundary, ax, MAP_RESOLUTION, goal, draw_path=draw_path)

    plt.show(block=True)

if __name__ == '__main__':
    parser = argparse.ArgumentParser(description='A* path planning')
    parser.add_argument('test_case', type=str, choices=['maze', 'single_cube', 'flappy_bird', 'monza', 'window', 'tower', 'room'], help='Test case name')
    parser.add_argument('--epsilon', type=float, default=1, help='Epsilon value for A*')
    args = parser.parse_args()

    if args.test_case == 'single_cube':
        start, goal, mapfile, test_case = test_single_cube() 
    elif args.test_case == 'maze':
        start, goal, mapfile, test_case = test_maze()
    elif args.test_case == 'flappy_bird':
        start, goal, mapfile, test_case = test_flappy_bird()
    elif args.test_case == 'monza':
        start, goal, mapfile, test_case = test_monza()
    elif args.test_case == 'window':
        start, goal, mapfile, test_case = test_window()
    elif args.test_case == 'tower':
        start, goal, mapfile, test_case = test_tower()
    main(start, goal, mapfile, test_case)
