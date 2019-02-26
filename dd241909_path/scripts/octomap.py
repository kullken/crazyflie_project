#!/usr/bin/env python

from __future__ import division

import math
import numpy as np

import rospy

def create_map(map_data, coll_rad, grid_unit_size=0.2, off_ground=0.2, side_margin=0.2):
    """Create map object dictionary map_dict."""

    airspace_min = Vec3(*map_data['airspace']['min'])
    airspace_max = Vec3(*map_data['airspace']['max'])

    # Convert obstacle to point cloud with finer granularity than map representation
    points = set()
    for wall in map_data['walls']:
        points.union(plane_to_points(wall['plane'], grid_unit_size, coll_rad))
    for gate in map_data['gates']:
        points.union(gate_to_points(gate, grid_unit_size, coll_rad, map_data['gate_size'], off_ground, side_margin, airspace_max.z))
    points = {point for point in points if airspace_min <= point <= airspace_max}

    # Adjust gate position to middle of gate for later convenience
    for gate in map_data['gates']:
        gate['width']  = map_data['gate_size'][0]
        gate['height'] = map_data['gate_size'][1]
        gate['position'] = Vec3(gate['position'][0], 
                                gate['position'][1], 
                                off_ground + gate['height']/2)

    return Octomap(points, map_data['gates'], map_data['markers'], map_data['airspace'], grid_unit_size)

def gate_to_points(gate, cube_side, coll_rad, gate_size, off_ground, side_margin, z_max):
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
    bottom_points = plane_to_points(bottom_plane, cube_side, coll_rad)

    top_start = bottom_start + Vec3(0.0, 0.0, off_ground+gate_height)
    top_stop  = bottom_stop + Vec3(0.0, 0.0, z_max-off_ground)
    top_plane = {'start': top_start, 'stop': top_stop}
    top_points = plane_to_points(top_plane, cube_side, coll_rad)

    side1_start = mid - parallel*gate_width/2 + Vec3(0.0, 0.0, off_ground)
    side1_stop  = mid - parallel*(side_margin + gate_width/2) + Vec3(0.0, 0.0, off_ground+gate_height)
    side1_plane = {'start': side1_start, 'stop': side1_stop}
    side1_points = plane_to_points(side1_plane, cube_side, coll_rad)

    side2_start = mid + parallel*gate_width/2 + Vec3(0.0, 0.0, off_ground)
    side2_stop  = mid + parallel*(side_margin + gate_width/2) + Vec3(0.0, 0.0, off_ground+gate_height)
    side2_plane = {'start': side2_start, 'stop': side2_stop}
    side2_points = plane_to_points(side2_plane, cube_side, coll_rad)

    points = set()
    points = points.union(bottom_points)
    points = points.union(top_points)
    points = points.union(side1_points)
    points = points.union(side2_points)
    return points

def plane_to_points(plane, cube_side, coll_rad):
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

    nr_zsteps  = round((zlen+2*coll_rad) / (cube_side/2) + 2)
    zlinspace  = np.linspace(-coll_rad,  zlen+coll_rad, nr_zsteps)

    nr_xysteps = round((xylen+2*coll_rad) / (cube_side/2) + 2)
    xylinspace = np.linspace(-coll_rad, xylen+coll_rad, nr_xysteps)

    nr_csteps  = round(2*coll_rad / (cube_side/2) + 2)
    clinspace  = np.linspace(-coll_rad, coll_rad, nr_csteps)

    points = set()
    for z in zlinspace:
        zpart = z * zvec
        for xy in xylinspace:
            xypart = xy * xyvec
            for c in clinspace:
                cpart = c * cvec
                try:
                    point = start + zpart + xypart + cpart
                except:
                    raise TypeError('Not compatible types: {} + {} + {} + {}'.format(type(start), type(zpart), type(xypart), type(cpart)))
                points.add(point)

    return points


class Octomap(object):
    """An occupancy grid map using an octree data structure under the hood."""

    def __init__(self, collision_points, gates, markers, airspace, grid_unit_size):
        self.gates          = gates
        self.markers        = markers
        self.airspace       = airspace
        self.grid_unit_size = grid_unit_size

        airspace_min = Vec3(*airspace['min'])
        airspace_max = Vec3(*airspace['max'])
        tree_center = (airspace_max + airspace_min) / 2
        diff = airspace_max - airspace_min
        tree_size = max(diff.x, diff.y, diff.z)

        # Calculate max depth for octree to achieve desired grid cube size
        max_depth = 0
        while grid_unit_size <= tree_size/(2**max_depth):
            max_depth += 1
        
        self.octree = Octree(tree_center, 
                             tree_size, 
                             collision_points, 
                             max_depth, 
                             airspace_min, 
                             airspace_max)

        return


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

        if bound_min is None:
            bound_min = Vec3(float('Inf'), float('Inf'), float('Inf'))
        if bound_max is None:
            bound_max = -Vec3(float('Inf'), float('Inf'), float('Inf'))

        tree_max = center + Vec3(size/2, size/2, size/2)
        tree_min = center - Vec3(size/2, size/2, size/2)

        if (not points) and (tree_min >= bound_min) and (tree_max <= bound_max):
            self.isleaf = True
            self.isfree = True
        elif max_depth == 0:
            self.isleaf = True
            self.isoccupied = True
        elif tree_max <= bound_min:
            self.isleaf = True
            self.isoccupied = True
        elif tree_min >= bound_max:
            self.isleaf = True
            self.isoccupied = True
        else:
            self.subtrees = []
            centers = self.find_subtree_centers(self.center, self.size)
            points_per_octant = self.split_by_octant(points, centers)
            diagonal = Vec3(size/4, size/4, size/4)

            for subcenter, subpoints in zip(centers, points_per_octant):
                sub_bound_min = bound_min
                sub_bound_max = bound_max
                if subcenter - diagonal >= bound_min:
                    sub_bound_min = None
                if subcenter + diagonal <= bound_max:
                    sub_bound_max = None

                self.subtrees.append(Octree(subcenter, size/2, subpoints, 
                                            max_depth-1, sub_bound_min, sub_bound_max))
        
        return

        
    def split_by_octant(self, points, center):
        points_by_octant = [set(), set(), set(), set(), 
                            set(), set(), set(), set()]
        for point in points:
            if   point.x >= center.x and point.y >= center.y and point.z >= center.z:
                points_by_octant[0].add(point)
            elif point.x  < center.x and point.y >= center.y and point.z >= center.z:
                points_by_octant[1].add(point)
            elif point.x  < center.x and point.y  < center.y and point.z >= center.z:
                points_by_octant[2].add(point)
            elif point.x >= center.x and point.y  < center.y and point.z >= center.z:
                points_by_octant[3].add(point)
            elif point.x >= center.x and point.y >= center.y and point.z  < center.z:
                points_by_octant[4].add(point)
            elif point.x  < center.x and point.y >= center.y and point.z  < center.z:
                points_by_octant[5].add(point)
            elif point.x  < center.x and point.y  < center.y and point.z  < center.z:
                points_by_octant[6].add(point)
            elif point.x >= center.x and point.y  < center.y and point.z  < center.z:
                points_by_octant[7].add(point)

        return points_by_octant

    def find_subtree_centers(self, center, size):
        centers = []
        for z in (size/4, -size/4):
            for y in (size/4, -size/4):
                for x in (size/4, -size/4):
                    centers.append(center + Vec3(x, y, z))
            
        return centers


    def query(points):
        """
        Input: 
            points - list of points to query

        Output:
            result - bool value of any point in points is colliding
        """
        if self.isleaf:
            return self.isoccupied
        else:
            points_per_octant = self.split_by_octant(points, centers)
            for octant, point_subset in zip(range(8), points_per_octant):
                if self.subtrees[octant].query(point_subset):
                    return True
            
            return False


class Vec3(object):
    __slots__ = '_x', '_y', '_z'

    # To hopefully override at least some numpy operations
    __array_priority__ = 100

    def __init__(self, x, y, z):
        self._x, self._y, self._z = x, y, z

    @property
    def x(self):
        return self._x
    @property
    def y(self):
        return self._y
    @property
    def z(self):
        return self._z

    def __getitem__(self, key):
        if key == 0:
            return self.x
        elif key == 1:
            return self.y
        elif key == 2:
            return self.z
        else:
            raise IndexError()

    def __repr__(self):
        return 'Vec3({}, {}, {})'.format(self.x, self.y, self.z)

    def __str__(self):
        # TODO: Round output
        return 'Vec3({}, {}, {})'.format(self.x, self.y, self.z)

    def __hash__(self):
        return hash((self.x, self.y, self.z))

    def __len__(self):
        return 3

    def __getitem__(self, key):
        if key == 0:
            return self.x
        elif key == 1:
            return self.y
        elif key == 2:
            return self.z
        else:
            raise IndexError()

    def __eq__(self, other):
        return self.x == other.x and self.y == other.y and self.z == other.z
    def __ne__(self, other):
        return self.x != other.x or self.y != other.y or self.z != other.z

    def __lt__(self, other):
        return self.x < other.x and self.y < other.y and self.z < other.z
    def __le__(self, other):
        return self.x <= other.x and self.y <= other.y and self.z <= other.z
    def __gt__(self, other):
        return self.x > other.x and self.y > other.y and self.z > other.z
    def __ge__(self, other):
        return self.x >= other.x and self.y >= other.y and self.z >= other.z

    def __abs__(self):
        return (self.x**2 + self.y**2 + self.z**2)**0.5

    def __neg__(self):
        return Vec3(-self.x, -self.y, -self.z)

    def __add__(self, other):
        return Vec3(self.x+other.x, self.y+other.y, self.z+other.z)
    def __radd__(self, other):
        return self.__add__(other)

    def __sub__(self, other):
        return Vec3(self.x-other.x, self.y-other.y, self.z-other.z)
    def __rsub__(self, other):
        return Vec3(other.x-self.x, other.y-self.y, other.z-self.z)

    def __mul__(self, other):
        return Vec3(self.x*other, self.y*other, self.z*other)
    def __rmul__(self, other):
        return self.__mul__(other)

    def __div__(self, other):
        return Vec3(self.x/other, self.y/other, self.z/other)
    def __truediv__(self, other):
        return Vec3(self.x/other, self.y/other, self.z/other)

    def unit(self):
        return self/abs(self)

    def dot(self, other):
        return self.x*other.x + self.y*other.y + self.z*other.z

    def cross(self, other):
        x = self.y*other.z - self.z*other.y
        y = self.z*other.x - self.x*other.z
        z = self.x*other.y - self.y*other.x
        return Vec3(x, y, z)


    






