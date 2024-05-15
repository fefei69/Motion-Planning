import numpy as np
import time
from skspatial.objects import Line, LineSegment, Plane, Points
# from skspatial.plotting import plot_3d
import matplotlib.pyplot as plt; plt.ion()
from mpl_toolkits.mplot3d import Axes3D
from mpl_toolkits.mplot3d.art3d import Poly3DCollection

def check_points_in_LineSegments(pt1, pt2, collision_points):
    line_seg = LineSegment(pt1, pt2)
    return line_seg.contains_point(collision_points)

def collision_checking(blocks, path, ax, verbose=False):
  '''
  check if the path collides with the blocks(obstacles) using AABBs and line segment intersection
  Treating path as a set of points and check if each points collide with blocks
  '''
  collision_points = []
  for block in blocks:
    # make 6 planes
    xmin, ymin, zmin = block[0], block[1], block[2]
    xmax, ymax, zmax = block[3], block[4], block[5]
    # fit a better plane using 
    # plane_front = Plane.best_fit([[xmin,ymin,zmax],[xmax,ymin,zmax],[xmin,ymin,zmin],[xmin,ymin,zmax]])
    # plane_top = Plane.best_fit([[xmin,ymin,zmax],[xmax,ymin,zmax],[xmin,ymax,zmax],[xmin,ymin,zmax]])
    # plane_back = Plane.best_fit([[xmin,ymax,zmax],[xmin,ymax,zmin],[xmax,ymax,zmax],[xmin,ymax,zmax]])
    # plane_left = Plane.best_fit([[xmin,ymin,zmax],[xmin,ymax,zmax],[xmin,ymin,zmin],[xmin,ymin,zmax]])
    # plane_right = Plane.best_fit([[xmax,ymin,zmax],[xmax,ymax,zmax],[xmax,ymin,zmin],[xmax,ymin,zmax]])
    # plane_bottom = Plane.best_fit([[xmax,ymin,zmin],[xmin,ymin,zmin],[xmax,ymax,zmin],[xmax,ymin,zmin]])

    plane_front = Plane.from_points([xmin,ymin,zmax], [xmax,ymin,zmax], [xmin,ymin,zmin])
    plane_top = Plane.from_points([xmin,ymin,zmax], [xmax,ymin,zmax], [xmin,ymax,zmax])
    plane_back = Plane.from_points([xmin,ymax,zmax], [xmin,ymax,zmin], [xmax,ymax,zmax])
    plane_left = Plane.from_points([xmin,ymin,zmax], [xmin,ymax,zmax], [xmin,ymin,zmin])
    plane_right = Plane.from_points([xmax,ymin,zmax], [xmax,ymax,zmax], [xmax,ymin,zmin])
    plane_bottom = Plane.from_points([xmax,ymin,zmin], [xmin,ymin,zmin], [xmax,ymax,zmin])
    planes = [plane_front, plane_top, plane_back, plane_left, plane_right, plane_bottom]

    for p in range(len(path)-1):
      point_intersections = []
      for plane in planes:
        plane_normal = plane.normal
        path_line = Line.from_points(path[p], path[p+1])
        path_line_dir = path_line.direction
        # skip collision checking if the plane and the path are parallel
        if plane_normal.is_perpendicular(path_line_dir):
          continue
        point_intersections.append(plane.intersect_line(path_line))
      
      for i in range(len(point_intersections)):
        pt_x = point_intersections[i][0]
        pt_y = point_intersections[i][1]
        pt_z = point_intersections[i][2]
        # AABBs
        if pt_x >= block[0] and pt_x<=block[3] and pt_y>=block[1] and pt_y<=block[4] and pt_z>=block[2] and pt_z<=block[5]: 
          # check if the point is in the line segment
          is_contained = check_points_in_LineSegments(path[p], path[p+1], np.array([pt_x,pt_y,pt_z]))
          if is_contained:
            collision_points.append(np.array([pt_x,pt_y,pt_z]))
            print("collision occur at",(pt_x,pt_y,pt_z))
  collision_points = np.array(collision_points)
  if verbose and len(collision_points) > 0:
    ax.plot(collision_points[:,0], collision_points[:,1], collision_points[:,2], 'g.')
  if len(collision_points) > 0:
    collided = True
  else:
    collided = False  
  return collision_points, collided

def tic():
  return time.time()
def toc(tstart, nm=""):
  print('%s took: %s sec.\n' % (nm,(time.time() - tstart)))
  

def load_map(fname):
  '''
  Loads the bounady and blocks from map file fname.
  
  boundary = [['xmin', 'ymin', 'zmin', 'xmax', 'ymax', 'zmax','r','g','b']]
  
  blocks = [['xmin', 'ymin', 'zmin', 'xmax', 'ymax', 'zmax','r','g','b'],
            ...,
            ['xmin', 'ymin', 'zmin', 'xmax', 'ymax', 'zmax','r','g','b']]
  '''
  mapdata = np.loadtxt(fname,dtype={'names': ('type', 'xmin', 'ymin', 'zmin', 'xmax', 'ymax', 'zmax','r','g','b'),\
                                    'formats': ('S8','f', 'f', 'f', 'f', 'f', 'f', 'f','f','f')})
  blockIdx = mapdata['type'] == b'block'
  boundary = mapdata[~blockIdx][['xmin', 'ymin', 'zmin', 'xmax', 'ymax', 'zmax','r','g','b']].view('<f4').reshape(-1,11)[:,2:]
  blocks = mapdata[blockIdx][['xmin', 'ymin', 'zmin', 'xmax', 'ymax', 'zmax','r','g','b']].view('<f4').reshape(-1,11)[:,2:]
  return boundary, blocks

def draw_map(boundary, blocks, start, goal):
  '''
  Visualization of a planning problem with environment boundary, obstacle blocks, and start and goal points
  '''
  fig = plt.figure()
  ax = fig.add_subplot(111, projection='3d')
  hb = draw_block_list(ax,blocks)
  hs = ax.plot(start[0:1],start[1:2],start[2:],'ro',markersize=7,markeredgecolor='k')
  hg = ax.plot(goal[0:1],goal[1:2],goal[2:],'go',markersize=7,markeredgecolor='k')  
  ax.set_xlabel('X')
  ax.set_ylabel('Y')
  ax.set_zlabel('Z')
  ax.set_xlim(boundary[0,0],boundary[0,3])
  ax.set_ylim(boundary[0,1],boundary[0,4])
  ax.set_zlim(boundary[0,2],boundary[0,5])
  return fig, ax, hb, hs, hg

def draw_block_list(ax,blocks):
  '''
  Subroutine used by draw_map() to display the environment blocks
  '''
  v = np.array([[0,0,0],[1,0,0],[1,1,0],[0,1,0],[0,0,1],[1,0,1],[1,1,1],[0,1,1]],dtype='float')
  f = np.array([[0,1,5,4],[1,2,6,5],[2,3,7,6],[3,0,4,7],[0,1,2,3],[4,5,6,7]])
  clr = blocks[:,6:]/255
  n = blocks.shape[0]
  d = blocks[:,3:6] - blocks[:,:3] 
  vl = np.zeros((8*n,3))
  fl = np.zeros((6*n,4),dtype='int64')
  fcl = np.zeros((6*n,3))
  for k in range(n):
    vl[k*8:(k+1)*8,:] = v * d[k] + blocks[k,:3]
    fl[k*6:(k+1)*6,:] = f + k*8
    fcl[k*6:(k+1)*6,:] = clr[k,:]
  
  if type(ax) is Poly3DCollection:
    ax.set_verts(vl[fl])
  else:
    pc = Poly3DCollection(vl[fl], alpha=0.25, linewidths=1, edgecolors='k')
    pc.set_facecolor(fcl)
    h = ax.add_collection3d(pc)
    return h