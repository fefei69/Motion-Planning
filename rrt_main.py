
import argparse
import time
import numpy as np
import matplotlib.pyplot as plt; plt.ion()
from rrt_algorithms.rrt import RRT
from rrt_algorithms.rrt_star_bid import RRTStarBidirectional
from rrt_algorithms.rrt_connect import RRTConnect
from rrt_algorithms.rrt_star import RRTStar
from rrt_algorithms.search_space import SearchSpace
from read_map import *
from utils import draw_map, draw_trees, collision_checking_robust, generate_planes

def main():
    parser = argparse.ArgumentParser(description="RRT* Path Planning")
    parser.add_argument('--alg', type=str, default='rrt_bid', 
                        choices=['rrt_bid', 'rrt_con', 'rrt_star','rrt'],
                        help='Choose a RRT* algorithm, default is RRT* bidirectional algorithm')
    parser.add_argument('--test_case', type=str, required=True, 
                        choices=['single_cube', 'maze', 'flappy_bird', 'monza', 'window', 'tower', 'room'],
                        help='Choose a test case for the RRT* algorithm')
    parser.add_argument('--q', type=float, default=1.0, help='Length of tree edges')
    parser.add_argument('--r', type=float, default=0.01, help='Length of smallest edge to check for intersection with obstacles')
    parser.add_argument('--max_samples', type=int, default=1024, help='Maximum number of samples before timing out')
    parser.add_argument('--rewire_count', type=int, default=32, help='Number of nearby branches to rewire')
    parser.add_argument('--prc', type=float, default=0.01, help='Probability of checking for a connection to goal')

    args = parser.parse_args()

    # Load the map based on the test case
    if args.test_case == 'single_cube':
        start, goal, mapfile, _ = test_single_cube()
    elif args.test_case == 'maze':
        start, goal, mapfile, _ = test_maze()
    elif args.test_case == 'flappy_bird':
        start, goal, mapfile, _ = test_flappy_bird()
    elif args.test_case == 'monza':
        start, goal, mapfile, _ = test_monza()
    elif args.test_case == 'window':
        start, goal, mapfile, _ = test_window()
    elif args.test_case == 'tower':
        start, goal, mapfile, _ = test_tower()
    elif args.test_case == 'room':
        start, goal, mapfile, _ = test_room()
    else:
        raise ValueError("Unknown test case")
    print("Path Planning Algorithm: ", args.alg)
    boundary, blocks = load_map(mapfile)
    Obstacles = blocks[:,0:6]
    X_dimensions = np.array([(boundary[0][0], boundary[0][3]), (boundary[0][1], boundary[0][4]), (boundary[0][2], boundary[0][5])])
    x_init = tuple(start)  # starting location
    x_goal = tuple(goal)  # goal location

    # Create Search Space
    X = SearchSpace(X_dimensions, Obstacles)
    start_time = time.time()
    # Initialize RRT* algorithm with parsed arguments
    if args.alg == 'rrt_bid':
        rrt = RRTStarBidirectional(X, args.q, x_init, x_goal, args.max_samples, args.r, args.prc, args.rewire_count)
        path = rrt.rrt_star_bidirectional()
    elif args.alg == 'rrt_con':
        rrt = RRTConnect(X, args.q, x_init, x_goal, args.max_samples, args.r)
        path = rrt.rrt_connect()
    elif args.alg == 'rrt_star':
        rrt = RRTStar(X, args.q, x_init, x_goal, args.max_samples, args.r, args.prc, args.rewire_count)
        path = rrt.rrt_star()
    elif args.alg == 'rrt':
        rrt = RRT(X, args.q, x_init, x_goal, args.max_samples, args.r, args.prc)
        path = rrt.rrt_search()
    else:
        raise ValueError("Unknown RRT* algorithm")
    end_time = time.time()
    if len(path) == 0:
        print("No path found")
        return None
    
    print("Drawing map.....")
    fig, ax, hb, hs, hg = draw_map(boundary, blocks, start, goal)
    collision_checking_robust(blocks, path, generate_planes(blocks), ax, verbose=True)
    print(f"Path Length: {np.sum(np.sqrt(np.sum(np.diff(path,axis=0)**2,axis=1)))}")
    print("Time taken: ", end_time - start_time)
    path = np.array(path)
    
   
    draw_trees(ax, rrt.trees)
    ax.plot(path[:,0], path[:,1], path[:,2], 'r-')
    plt.show(block=True)
if __name__ == '__main__':
    main()
