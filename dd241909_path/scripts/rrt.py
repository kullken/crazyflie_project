#!/usr/bin/env python

import math
import numpy as np
from collections import namedtuple

import rospy

from geometry import Vec3


class RRT(object):

    _seed = np.random.randint(1000, 9999)

    def __init__(self, map):
        self.map = map
        
        rospy.loginfo('RRT: Initialising with random seed {}'.format(self._seed))
        np.random.seed(self._seed)

        # Access rrt ros parameters
        self.wp_tol             = rospy.get_param(rospy.get_namespace() + 'rrt/waypoint_tolerance')
        self.goal_sample_prob   = rospy.get_param(rospy.get_namespace() + 'rrt/goal_sample_prob')

    def plan_path(self, start_wp, waypoints):
        path = self.plan_single_path(start_wp, waypoints[0])
        for start, goal in zip(waypoints[0:-1], waypoints[1:]):
            path.extend(self.plan_single_path(start, goal)[1:])
        return path

    def plan_single_path(self, start_wp, goal_wp):
        self.first_sample = True

        start = self.wp_to_node(start_wp)
        goal = self.wp_to_node(goal_wp)

        start.parent = None
        tree = RRTTree(start)
        
        for iter in xrange(1000):

            rospy.loginfo_throttle(1, 'RRT: Iteration {}'.format(iter))

            randnode = self.sample_node(goal)
            nearnode = tree.find_nearest(randnode)
            newnode = self.steer(nearnode, randnode)
            if not newnode:
                continue

            tree.add(newnode, parent=nearnode)
            tree.rewire(newnode)

            if self.reached_goal(newnode, goal):
                rospy.loginfo('RRT: Goal reached!')
                rospy.loginfo('RRT: {} total iterations'.format(iter))
                return self.retrace_path(newnode)
        
        rospy.logwarn('RRT: Failed!')
        rospy.loginfo('RRT: {} total iterations'.format(iter))

        return []

    def sample_node(self, goal):
        # Always sample goal first time
        if self.first_sample:
            self.first_sample = False
            return goal
        elif np.random.rand() < self.goal_sample_prob:
            return goal
        else:
            rand = np.random.rand(3)
            min = np.array(self.map.airspace_min)
            max = np.array(self.map.airspace_max)
            pos = Vec3(*(rand*max + (1-rand)*min))
            #yaw = np.random.rand() * 2*math.pi
            yaw = 0.0
            return RRTNode(pos, yaw)

    def steer(self, initnode, targetnode):
        start = initnode.pos
        end = targetnode.pos
        points = [start*t + end*(1-t) for t in np.linspace(0, 1)]
        if self.map.query(points):
            return
        else:
            return targetnode

    def reached_goal(self, node, goal):
        return abs(node.pos - goal.pos) <= self.wp_tol

    def retrace_path(self, node):
        Waypoint = namedtuple('Waypoint', ('x', 'y', 'z', 'yaw'))
        path = []
        while not node is None:
            path.append(Waypoint(node.pos.x, node.pos.y, node.pos.z, node.yaw))
            node = node.parent
        path.reverse()
        return path

    def wp_to_node(self, wp):
        return RRTNode(Vec3(wp.x, wp.y, wp.z), wp.yaw)
        

class RRTTree(object):

    def __init__(self, root):
        self.root = root
        self.nodes = [root]

    def add(self, newnode, parent):
        newnode.parent = parent
        self.nodes.append(newnode)

    def find_nearest(self, node):
        # TODO: kd-tree
        min_dist = float('inf')
        min_neighbour = None
        for neighbour in self.nodes:
            dist = abs(neighbour.pos - node.pos)
            if dist <= min_dist:
                min_dist = dist
                min_neighbour = neighbour
        return min_neighbour

    def rewire(self, newnode):
        # TODO: Look for nodes which gets lower cost passing through newnode
        pass


class RRTNode(object):

    def __init__(self, pos, yaw):
        self._pos = pos
        self._yaw = yaw 

    def __repr__(self):
        return 'Node({}, {}, {}, {})'.format(self.pos.x, self.pos.y, self.pos.z, self.yaw)

    def __hash__(self):
        return hash(self.key)
    def __eq__(self, other):
        return self.key == other.key
    def __ne__(self, other):
        return self.key != other.key

    @property
    def key(self):
        return (self.pos, self.yaw)
    @property
    def pos(self):
        return self._pos
    @property
    def yaw(self):
        return self._yaw