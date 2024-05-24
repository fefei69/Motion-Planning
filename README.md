# ECE276B PR2 Spring 2024

## Overview
In this assignment, you will implement and compare the performance of search-based and sampling-based motion planning algorithms on several 3D environments.
## Searh-based method - A*
```
python astar_main.py single_cube --epsilon 1
```
## Weighted A*
```
python astar_main.py single_cube --epsilon 1.5
```
## Sampling-based method 
- RRT*
- RRT-connected
- Bidirectional RRT* (default)
- RRT 
```
python rrt_main.py --test_case maze
```
## For the Maze Case you might want to increase max sample size:
```
python rrt_main.py --alg rrt_star --test_case maze --max_samples 100000

```