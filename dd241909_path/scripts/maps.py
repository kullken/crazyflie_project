#!/usr/bin/env python

from __future__ import division

import math
import copy
import Queue
import numpy as np
from itertools import count

import rospy
from geometry_msgs.msg import Point
from visualization_msgs.msg import Marker, MarkerArray

#from geometry import Vec3, Line, Bezier

from myprofiling import profile

import pyximport
pyximport.install()
from cgeometry import Vec3, Line, Bezier


def create_map(map_data, type='grid'):
    """Create map object dictionary map_dict."""

    # Access map parameters through ros
    coll_rad    = rospy.get_param(rospy.get_namespace() + 'crazyflie/collision_radius')
    resolution  = rospy.get_param(rospy.get_namespace() + 'map/resolution')
    off_ground  = rospy.get_param(rospy.get_namespace() + 'map/gate_off_ground')
    side_margin = rospy.get_param(rospy.get_namespace() + 'map/gate_side_margin')

    airspace_min = Vec3(*map_data['airspace']['min'])
    #airspace_max = Vec3(*map_data['airspace']['max'])
    airspace_max = Vec3(map_data['airspace']['max'][0], map_data['airspace']['max'][1], 1.0)

    # Convert obstacle to point cloud with finer resolution than map representation
    points = set()
    for wall in map_data['walls']:
        points = points.union(plane_to_points(wall['plane'], resolution, coll_rad))
    for gate in map_data['gates']:
        points = points.union(gate_to_points(gate, resolution, coll_rad, map_data['gate_size'], off_ground, side_margin, airspace_max.z))
    points = {point for point in points if (airspace_min <= point <= airspace_max)}

    # Adjust gate position to middle of gate for later convenience
    for gate in map_data['gates']:
        gate['width']  = map_data['gate_size'][0]
        gate['height'] = map_data['gate_size'][1]
        gate['position'] = Vec3(gate['position'][0], 
                                gate['position'][1], 
                                off_ground + gate['height']/2)

    # Create map of specified by type
    if type == 'kd':
        map = Kdmap(points, map_data['gates'], map_data['markers'], airspace_min, airspace_max, resolution)
    elif type == 'octo':
        map = Octomap(points, map_data['gates'], map_data['markers'], airspace_min, airspace_max, resolution)
    else:
        map = Gridmap(points, map_data['gates'], map_data['markers'], airspace_min, airspace_max, resolution)

    # Publish map shapes to rviz
    if rospy.get_param(rospy.get_name() + '/use_rviz'):
        publish_map_to_rviz(map, points)
    
    return map

def gate_to_points(gate, resolution, coll_rad, gate_size, off_ground, side_margin, z_max):
    gate_width  = gate_size[0]
    gate_height = gate_size[1]

    mid = Vec3(*gate['position'])
    
    theta = gate['heading'] * 2*math.pi/360
    normal = Vec3(math.cos(theta), math.sin(theta), 0.0)
    parallel = normal.cross(Vec3(0.0, 0.0, 1.0))

    # Create four planes to represent gate w.r.t. collision
    bottom_start = mid - parallel*(side_margin + gate_width/2)
    bottom_stop  = mid + parallel*(side_margin + gate_width/2) + Vec3(0.0, 0.0, off_ground)
    bottom_plane = {'start': bottom_start, 'stop': bottom_stop}
    bottom_points = plane_to_points(bottom_plane, resolution, coll_rad)

    top_start = bottom_start + Vec3(0.0, 0.0, off_ground+gate_height)
    top_stop  = bottom_stop + Vec3(0.0, 0.0, z_max-off_ground)
    top_plane = {'start': top_start, 'stop': top_stop}
    top_points = plane_to_points(top_plane, resolution, coll_rad)

    side1_start = mid - parallel*gate_width/2 + Vec3(0.0, 0.0, off_ground)
    side1_stop  = mid - parallel*(side_margin + gate_width/2) + Vec3(0.0, 0.0, off_ground+gate_height)
    side1_plane = {'start': side1_start, 'stop': side1_stop}
    side1_points = plane_to_points(side1_plane, resolution, coll_rad)

    side2_start = mid + parallel*gate_width/2 + Vec3(0.0, 0.0, off_ground)
    side2_stop  = mid + parallel*(side_margin + gate_width/2) + Vec3(0.0, 0.0, off_ground+gate_height)
    side2_plane = {'start': side2_start, 'stop': side2_stop}
    side2_points = plane_to_points(side2_plane, resolution, coll_rad)

    points = set()
    points = points.union(bottom_points)
    points = points.union(top_points)
    points = points.union(side1_points)
    points = points.union(side2_points)
    return points

def plane_to_points(plane, resolution, coll_rad):
    start = Vec3(*plane['start'])
    stop  = Vec3(*plane['stop'])

    xyvec = Vec3(stop.x - start.x, stop.y - start.y, 0.0)   # Plane vector in (xy)-plane
    zvec  = Vec3(0.0, 0.0, stop.z - start.z)                # Plane vector along (z)-axis
    cvec  = zvec.cross(xyvec)                               # Plane normal vector, used for (c)ollision map inflation

    xylen = abs(xyvec)
    zlen  = abs(zvec)

    xyvec = xyvec.unit()
    zvec  = zvec.unit()
    cvec  = cvec.unit()

    nr_zsteps  = round((zlen+2*coll_rad) / (resolution/2) + 2)
    zlinspace  = np.linspace(-coll_rad,  zlen+coll_rad, nr_zsteps)

    nr_xysteps = round((xylen+2*coll_rad) / (resolution/2) + 2)
    xylinspace = np.linspace(-coll_rad, xylen+coll_rad, nr_xysteps)

    nr_csteps  = round(2*coll_rad / (resolution/2) + 2)
    clinspace  = np.linspace(-coll_rad, coll_rad, nr_csteps)

    points = set()
    for z in zlinspace:
        zpart = z * zvec
        start_zpart = start + zpart
        for xy in xylinspace:
            xypart = xy * xyvec
            start_zxypart = start_zpart + xypart
            for c in clinspace:
                cpart = c * cvec
                point = start_zxypart + cpart
                points.add(point)

    return points


class Gridmap(object):

    def __init__(self, collision_points, gates, markers, min, max, resolution):
        self.gates      = gates
        self.markers    = markers
        self.min        = min
        self.max        = max
        self.resolution = resolution
        self.origin     = self.min
        
        # Initialise grid with false values
        max_index = self._point_to_index(self.max)
        self.shape = (max_index[0]+1, max_index[1]+1, max_index[2]+1)
        self.grid = np.full(self.shape, False, dtype=np.bool)

        for point in collision_points:
            index = self._point_to_index(point)
            try:
                self.grid[index] = True
            except IndexError:
                rospy.logwarn('IndexError in Gridmap.__init__()')

    def query(self, item):
        if isinstance(item, list):
            return self._query_points(item)
        elif isinstance(item, Line):
            return self._query_line(item)
        elif isinstance(item, Bezier):
            return self._query_bezier(item)
        else:
            raise TypeError('Type not supported for collision queries in Gridmap.')

    def _query_points(self, points):
        for point in points:
            if not self.min <= point <= self.max:
                return True
            index = self._point_to_index(point)
            if self.grid[index]:
                return True
        return False

    def _query_line(self, line):
        start = line.p1
        diff = line.p2 - line.p1
        line_len = abs(line)
        direction = diff / line_len
        nrsamples = round(line_len * 50 + 1)     # Distance between samples becomes approx 0.02 m
        points = [start + direction*t for t in np.linspace(0, line_len, nrsamples)]
        return self.query_points(points)

    def _query_bezier(self, bezier):
        nrsamples = int(bezier.T / 0.1)
        t_samples = np.linspace(0, bezier.T, nrsamples)

        # Test slices seperately
        step = 10
        for i in range(step):
            t_slice = t_samples[i::step]
            points = [bezier.pos(t) for t in t_slice]
            if self._query_points(points):
                return True

        return False

    def _point_to_index(self, point):
        point = (point - self.origin) / self.resolution
        return int(math.floor(point.x)), int(math.floor(point.y)), int(math.floor(point.z))


class Octomap(object):
    """An occupancy grid map using an octree data structure under the hood."""

    def __init__(self, collision_points, gates, markers,  min, max, resolution):
        self.gates      = gates
        self.markers    = markers
        self.min        = min
        self.max        = max
        self.resolution = resolution

        tree_center = (self.max + self.min) / 2
        diff = self.max - self.min
        tree_size = max(diff.x, diff.y, diff.z)

        # Calculate max depth for octree to achieve desired grid cube size
        max_depth = 0
        while resolution <= tree_size/(2**max_depth):
            max_depth += 1
        
        self.octree = Octree(tree_center, 
                             tree_size, 
                             collision_points, 
                             max_depth, 
                             self.min, 
                             self.max)

        return

    def query(self, item):
        if isinstance(item, list):
            return self.octree.query_points(item)
        elif isinstance(item, Line):
            return self.octree.query_line(item)
        elif isinstance(item, Bezier):
            return self.octree.query_bezier(item)
        else:
            raise TypeError('Type not supported for collision queries in Octomap.')
        

class Octree(object):
    """
    An octree data structure for creating 3D occupancy grid 
    grid maps from point cloud obstacles. All points are 
    regarded as absolutely true.
    
    Subtrees are numbered as zero-indexed octants.
        ____________
        |    x y z |
        | 0: + + + |
        | 1: - + + |
        | 2: - - + |
        | 3: + - + |
        | 4: + + - |
        | 5: - + - |
        | 6: - - - |
        | 7: + - - |
        |__________|

    """

    def __init__(self, center, size, points, max_depth, bound_min=None, bound_max=None):
        self.center     = center
        self.size       = size
        self.isleaf     = False
        self.isfree     = False
        self.isoccupied = False
        self.subtrees = []

        if bound_min is None:
            bound_min = -Vec3(float('Inf'), float('Inf'), float('Inf'))
        if bound_max is None:
            bound_max = Vec3(float('Inf'), float('Inf'), float('Inf'))

        tree_max = center + Vec3(size/2, size/2, size/2)
        tree_min = center - Vec3(size/2, size/2, size/2)

        if (not points) and (tree_min >= bound_min) and (tree_max <= bound_max):
            self.isleaf = True
            self.isfree = True
        elif max_depth == 0:
            self.isleaf = True
            self.isoccupied = True
        elif tree_max.x <= bound_min.x or tree_max.y <= bound_min.y or tree_max.z <= bound_min.z:
            self.isleaf = True
            self.isoccupied = True
        elif tree_min.x >= bound_max.x or tree_min.y >= bound_max.y or tree_min.z >= bound_max.z:
            self.isleaf = True
            self.isoccupied = True
        else:
            centers = self.find_subtree_centers(self.center, self.size)
            points_per_octant = self.split_by_octant(points, self.center)
            diagonal = Vec3(size/4, size/4, size/4)

            # Generate subtrees
            for subcenter, subpoints in zip(centers, points_per_octant):
                sub_bound_min = bound_min
                sub_bound_max = bound_max
                if subcenter - diagonal >= bound_min:
                    sub_bound_min = None
                if subcenter + diagonal <= bound_max:
                    sub_bound_max = None

                self.subtrees.append(Octree(subcenter, size/2, subpoints, 
                                            max_depth-1, sub_bound_min, sub_bound_max))

            # Remove subtrees if all turn out to be occupied
            for subtree in self.subtrees:
                if not subtree.isoccupied:
                    break
            else:
                self.isleaf = True
                self.isoccupied = True
                self.subtrees = []

        assert not (self.isfree and self.isoccupied)
        
        return

    def split_by_octant(self, points, center):
        points_by_octant = [set(), set(), set(), set(), 
                            set(), set(), set(), set()]
        for point in points:
            if point.z >= center.z:
                if point.y >= center.y:
                    if point.x >= center.x:
                        points_by_octant[0].add(point)
                    else:
                        points_by_octant[1].add(point)
                else:
                    if point.x  < center.x:
                        points_by_octant[2].add(point)
                    else:
                        points_by_octant[3].add(point)
            else:
                if point.y >= center.y:
                    if point.x >= center.x:
                        points_by_octant[4].add(point)
                    else:
                        points_by_octant[5].add(point)
                else:
                    if point.x  < center.x:
                        points_by_octant[6].add(point)
                    else:
                        points_by_octant[7].add(point)

        return points_by_octant

    def find_subtree_centers(self, center, size):
        directions = [Vec3( 1,  1,  1),
                      Vec3(-1,  1,  1),
                      Vec3(-1, -1,  1),
                      Vec3( 1, -1,  1),
                      Vec3( 1,  1, -1),
                      Vec3(-1,  1, -1),
                      Vec3(-1, -1, -1),
                      Vec3( 1, -1, -1)]

        return [center + direct*size/4 for direct in directions]

    def query_points(self, points):
        """
        Input: 
            points: iterable - points to query

        Output:
            result: bool - whether any queried points are colliding
        """
        if not points:
            return False
        elif self.isleaf:
            return self.isoccupied
        else:
            points_per_octant = self.split_by_octant(points, self.center)
            for subtree, point_subset in zip(self.subtrees, points_per_octant):
                if subtree.query_points(point_subset):
                    return True
            return False

    def query_line(self, line):
        start = line.p1
        diff = line.p2 - line.p1
        line_len = abs(line)
        direction = diff / line_len
        nrsamples = round(line_len * 50 + 1)     # Distance between samples becomes approx 0.02 m
        points = [start + direction*t for t in np.linspace(0, line_len, nrsamples)]
        return self.query_points(points)

    def query_bezier(self, bezier):
        nrsamples = int(bezier.T / 0.1)
        points = [bezier.pos(t) for t in np.linspace(0, bezier.T, nrsamples)]
        return self.query_points(points)


class Kdmap(object):

    def __init__(self, collision_points, gates, markers,  min, max, resolution):
        self.gates      = gates
        self.markers    = markers
        self.min        = min
        self.max        = max
        self.resolution = resolution
        
        self.kdtree = Kdtree(
                self.min,
                self.max,
                collision_points,
                resolution
        )
        
        return

    def query(self, item):
        if type(item) == list:
            for point in item:
                if not self.min < point < self.max:
                    return True
            return self.kdtree.query(item)
        else:
            raise TypeError('Type not supported for collision queries in Kdmap.')

    
class Kdtree(object):

    def __init__(self, min, max, points, resolution):
        self.min        = min
        self.max        = max
        self.isleaf     = False
        self.isfree     = False
        self.isoccupied = False
        self.low_tree   = None
        self.high_tree  = None

        diag = self.max - self.min

        # No points, then we are done
        if not points:
            self.isleaf = True
            self.isfree = True
            return
        # Check if all dimensions are smaller than resolution
        elif diag <= Vec3(resolution, resolution, resolution):
            self.isleaf = True
            self.isoccupied = True
            return

        # Not a leaf so we have to actually do something
        (self.split_dim, 
         self.split_value, 
         low_points, 
         high_points) = self.find_split(points, resolution)
        
        low_min = Vec3(*self.min)
        low_max_values = [self.max.x, self.max.y, self.max.z]
        low_max_values[self.split_dim] = self.split_value
        low_max = Vec3(*low_max_values)
        self.low_tree = Kdtree(low_min, low_max, low_points, resolution)

        high_min_values = [self.min.x, self.min.y, self.min.z]
        high_min_values[self.split_dim] = self.split_value
        high_min = Vec3(*high_min_values)
        high_max = Vec3(*self.max)
        self.high_tree = Kdtree(high_min, high_max, high_points, resolution)

        # Remove subtrees if both turn out to be occupied
        if self.low_tree.isoccupied and self.high_tree.isoccupied:
            self.low_tree = None
            self.high_tree = None
            self.isleaf = True
            self.isoccupied = True

        return

    def find_split(self, points, resolution):
        split_dim = int(np.argmax(self.max - self.min))
        points = sorted(points, key=lambda p: p[split_dim])

        mid_value = ((self.max + self.min) / 2)[split_dim]
        # No slices thinner than resolution/2
        min_value = min(points[0][split_dim], self.max[split_dim] - resolution/2)
        max_value = max(points[-1][split_dim], self.min[split_dim] + resolution/2)

        # Split by sliding midpoint
        if min_value >= mid_value:
            return split_dim, min_value, [], points
        if max_value <= mid_value:
            return split_dim, max_value, points, []
        
        # split_i = -1
        # for i, point in enumerate(points):
        #     if point[split_dim] >= mid_value:
        #         split_i = i
        #         break
        
        # return split_dim, mid_value, points[0:split_i], points[split_i:-1]
        low_points = [point for point in points if point[split_dim] <= mid_value]
        high_points = [point for point in points if point[split_dim] > mid_value]
        return split_dim, mid_value, low_points, high_points

    def query_points(self, points):
        """
        Input: 
            points: iterable - points to query

        Output:
            result: bool - whether any queried points are colliding
        """
        if not points:
            return False
        elif self.isleaf:
            return self.isoccupied
        else:
            low_points = [point for point in points if point[self.split_dim] <= self.split_value]
            if self.low_tree.query_points(low_points):
                return True
            high_points = [point for point in points if point[self.split_dim] >= self.split_value]
            if self.high_tree.query_points(high_points):
                return True
            return False


# Visualisation and publishing functions for rviz
def publish_map_to_rviz(map, points):
    rviz_marker_pub = rospy.Publisher('/map_marker_array', MarkerArray, queue_size=2)

    # Does this delete all markers on all marker topics or only this topic?
    # Clear all previous rviz markers
    delete_msg = MarkerArray()
    delete_marker = Marker()
    delete_marker.action = Marker.DELETEALL
    delete_msg.markers = [delete_marker]
    rviz_marker_pub.publish(delete_msg)

    rospy.sleep(0.3)

    id_gen = count()

    if isinstance(map, Gridmap):
        markers = gridmap_to_msgs(map, id_gen)
    elif isinstance(map, Octomap):
        markers = octomap_to_msgs(map, id_gen)
    elif isinstance(map, Kdmap):
        markers = kdmap_to_msgs(map, id_gen)
    else:
        raise TypeError('Map type does not support rviz visualisation!')

    markers.append(points_to_msg(points, id_gen))

    marker_array = MarkerArray()
    marker_array.markers = markers
    
    rospy.sleep(0.3)

    rviz_marker_pub.publish(marker_array)

    return

def gridmap_to_msgs(map, id_gen):
    marker = Marker()
    marker.header.frame_id = 'map'
    marker.ns = 'collision_cubes'
    marker.id = next(id_gen)
    marker.type = Marker.CUBE_LIST
    marker.action = Marker.ADD
    marker.color.r = 1.0
    marker.color.g = 0.0
    marker.color.b = 0.0
    marker.color.a = 1.0
    marker.scale.x = map.resolution
    marker.scale.y = map.resolution
    marker.scale.z = map.resolution
    marker.points = []
    for xi in range(map.shape[0]):
        for yi in range(map.shape[1]):
            for zi in range(map.shape[2]):
                if map.grid[xi, yi, zi]:
                    p = Vec3(xi+0.5, yi+0.5, zi+0.5) * map.resolution + map.origin
                    marker.points.append(Point(x=p.x, y=p.y, z=p.z))
        
    return [marker]

def kdmap_to_msgs(map, id_gen):
    markers = []
    queue = Queue.Queue()
    queue.put(map.kdtree)
    while not queue.empty():
        tree = queue.get()
        if tree.isoccupied:
            mid_point = (tree.max + tree.min) / 2
            size = tree.max - tree.min
            marker = Marker()
            marker.header.frame_id = 'map'
            marker.ns = 'collision_cubes'
            marker.id = next(id_gen)
            marker.type = Marker.CUBE
            marker.action = Marker.ADD
            marker.color.r = 1.0
            marker.color.g = 0.0
            marker.color.b = 0.0
            marker.color.a = 0.6
            marker.pose.position.x = mid_point.x
            marker.pose.position.y = mid_point.y
            marker.pose.position.z = mid_point.z
            marker.scale.x = size.x
            marker.scale.y = size.y
            marker.scale.z = size.z
            markers.append(marker)
        elif tree.isfree:
            mid_point = (tree.max + tree.min) / 2
            size = tree.max - tree.min
            marker = Marker()
            marker.header.frame_id = 'map'
            marker.ns = 'free_cubes'
            marker.id = next(id_gen)
            marker.type = Marker.CUBE
            marker.action = Marker.ADD
            marker.color.r = 0.0
            marker.color.g = 1.0
            marker.color.b = 0.0
            marker.color.a = 0.2
            marker.pose.position.x = mid_point.x
            marker.pose.position.y = mid_point.y
            marker.pose.position.z = mid_point.z
            marker.scale.x = size.x
            marker.scale.y = size.y
            marker.scale.z = size.z
            markers.append(marker)
        else:
            queue.put(tree.low_tree)
            queue.put(tree.high_tree)

    return markers

def octomap_to_msgs(map, id_gen):
    occu_size2cube_map = {}
    free_size2cube_map = {}
    queue = Queue.Queue()
    queue.put(map.octree)
    while not queue.empty():
        tree = queue.get()
        if tree.isoccupied:
            if not map.min < tree.center < map.max:
                continue
            if not tree.size in occu_size2cube_map:
                occu_size2cube_map[tree.size] = set()
            occu_size2cube_map[tree.size].add(tree.center)
        elif tree.isfree:
            if not map.min < tree.center < map.max:
                continue
            if not tree.size in free_size2cube_map:
                free_size2cube_map[tree.size] = set()
            free_size2cube_map[tree.size].add(tree.center)
        else:
            for subtree in tree.subtrees:
                queue.put(subtree)

    markers = []

    for size, centers in occu_size2cube_map.items():
        marker = Marker()
        marker.header.frame_id = 'map'
        marker.ns = 'collision_cubes'
        marker.id = next(id_gen)
        marker.type = Marker.CUBE_LIST
        marker.action = Marker.ADD
        marker.color.r = 1.0
        marker.color.g = 0.0
        marker.color.b = 0.0
        marker.color.a = 0.6
        marker.scale.x = size
        marker.scale.y = size
        marker.scale.z = size
        marker.points = []
        for c in centers:
            marker.points.append(Point(x=c.x, y=c.y, z=c.z))
        markers.append(marker)

    for size, centers in free_size2cube_map.items():
        marker = Marker()
        marker.header.frame_id = 'map'
        marker.ns = 'free_cubes'
        marker.id = next(id_gen)
        marker.type = Marker.CUBE_LIST
        marker.action = Marker.ADD
        marker.color.r = 0.0
        marker.color.g = 1.0
        marker.color.b = 0.0
        marker.color.a = 0.2
        marker.scale.x = size
        marker.scale.y = size
        marker.scale.z = size
        marker.points = []
        for c in centers:
            marker.points.append(Point(x=c.x, y=c.y, z=c.z))
        markers.append(marker)

    return markers

def points_to_msg(points, id_gen):
    marker = Marker()
    marker.header.frame_id = 'map'
    marker.ns = 'collision_points'
    marker.id = next(id_gen)
    marker.type = Marker.SPHERE_LIST
    marker.action = Marker.ADD
    marker.color.r = 0.0
    marker.color.g = 0.0
    marker.color.b = 1.0
    marker.color.a = 0.5
    marker.scale.x = 0.05
    marker.scale.y = 0.05
    marker.scale.z = 0.05
    marker.points = []
    for p in points:
        marker.points.append(Point(x=p.x, y=p.y, z=p.z))
        
    return marker








