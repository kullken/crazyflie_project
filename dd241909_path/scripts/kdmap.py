#!/usr/bin/env python

from __future__ import division

import math
import numpy as np

import rospy

from geometry import Vec3
from octomap import gate_to_points, plane_to_points


def create_kdmap(map_data, coll_rad, grid_unit_size=0.1, off_ground=0.2, side_margin=0.1):
    """Create map object dictionary map_dict."""

    airspace_min = Vec3(*map_data['airspace']['min'])
    airspace_max = Vec3(*map_data['airspace']['max'])

    # Convert obstacle to point cloud with finer granularity than map representation
    points = set()
    for wall in map_data['walls']:
        points = points.union(plane_to_points(wall['plane'], grid_unit_size, coll_rad))
    for gate in map_data['gates']:
        points = points.union(gate_to_points(gate, grid_unit_size, coll_rad, map_data['gate_size'], off_ground, side_margin, airspace_max.z))
    points = {point for point in points if (airspace_min <= point <= airspace_max)}

    # Adjust gate position to middle of gate for later convenience
    for gate in map_data['gates']:
        gate['width']  = map_data['gate_size'][0]
        gate['height'] = map_data['gate_size'][1]
        gate['position'] = Vec3(gate['position'][0], 
                                gate['position'][1], 
                                off_ground + gate['height']/2)

    # fewer_points = set()
    # for i, p in zip(range(1), points):
    #     fewer_points.add(p)
    # points = fewer_points

    # Including points in return for visual debugging in rviz
    return Kdmap(points, map_data['gates'], map_data['markers'], map_data['airspace'], grid_unit_size), points


class Kdmap(object):

    def __init__(self, collision_points, gates, markers, airspace, grid_unit_size):
        self.gates          = gates
        self.markers        = markers
        self.airspace_min   = Vec3(*airspace['min'])
        self.airspace_max   = Vec3(*airspace['max'])
        self.grid_unit_size = grid_unit_size
        
        self.kdtree = Kdtree(
                self.airspace_min,
                self.airspace_max,
                collision_points,
                grid_unit_size
        )

        return

    def query(self, points):
        for point in points:
            if not self.airspace_min < point < self.airspace_max:
                return True
        return self.kdtree.query(points)

    
class Kdtree(object):

    def __init__(self, min, max, points, stop_size):
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
        # Check if all dimensions are smaller than stop_size
        elif diag <= Vec3(stop_size, stop_size, stop_size):
            self.isleaf = True
            self.isoccupied = True
            return

        # Not a leaf so we have to actually do something
        (self.split_dim, 
         self.split_value, 
         low_points, 
         high_points) = self.find_split(points, stop_size)
        
        low_min = Vec3(*self.min)
        low_max_values = [self.max.x, self.max.y, self.max.z]
        low_max_values[self.split_dim] = self.split_value
        low_max = Vec3(*low_max_values)
        self.low_tree = Kdtree(low_min, low_max, low_points, stop_size)

        high_min_values = [self.min.x, self.min.y, self.min.z]
        high_min_values[self.split_dim] = self.split_value
        high_min = Vec3(*high_min_values)
        high_max = Vec3(*self.max)
        self.high_tree = Kdtree(high_min, high_max, high_points, stop_size)

        # Remove subtrees if both turn out to be occupied
        if self.low_tree.isoccupied and self.high_tree.isoccupied:
            self.low_tree = None
            self.high_tree = None
            self.isleaf = True
            self.isoccupied = True

        return

    def find_split(self, points, stop_size):
        split_dim = int(np.argmax(self.max - self.min))
        points = sorted(points, key=lambda p: p[split_dim])

        mid_value = ((self.max + self.min) / 2)[split_dim]
        # No slices thinner than stop_size/2
        min_value = min(points[0][split_dim], self.max[split_dim] - stop_size/2)
        max_value = max(points[-1][split_dim], self.min[split_dim] + stop_size/2)

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

    def query(self, points):
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
            if self.low_tree.query(low_points):
                return True
            high_points = [point for point in points if point[self.split_dim] >= self.split_value]
            if self.high_tree.query(high_points):
                return True
            return False


