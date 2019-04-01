#!/usr/bin/env python

from __future__ import division

import math
import numpy as np
from collections import namedtuple, defaultdict

import rospy
from geometry_msgs.msg import Point
from visualization_msgs.msg import Marker

from geometry import Vec3, Line, Bezier, Waypoint, isclose

from myprofiling import profile


class RRT(object):

    _seed = np.random.randint(1000, 9999)
    _seed = 4865

    def __init__(self, map):
        self.map = map
        
        rospy.loginfo('RRT: Initialising with random seed {}'.format(self._seed))
        np.random.seed(self._seed)

        # Access rrt ros parameters
        self.iterations         = rospy.get_param(rospy.get_namespace() + 'rrt/iterations')
        self.goal_sample_prob   = rospy.get_param(rospy.get_namespace() + 'rrt/goal_sample_prob')
        #self.max_step_len       = rospy.get_param(rospy.get_namespace() + 'rrt/max_step_length')
        self.min_step_time      = rospy.get_param(rospy.get_namespace() + 'rrt/min_step_time')
        self.max_step_time      = rospy.get_param(rospy.get_namespace() + 'rrt/max_step_time')
        self.wp_pos_tol         = rospy.get_param(rospy.get_namespace() + 'rrt/waypoint_position_tolerance')
        self.wp_vel_tol         = rospy.get_param(rospy.get_namespace() + 'rrt/waypoint_velocity_tolerance')
        self.vel_max            = rospy.get_param(rospy.get_namespace() + 'crazyflie/vel_max')
        self.acc_max            = rospy.get_param(rospy.get_namespace() + 'crazyflie/acc_max')

    def plan_path(self, start_wp, waypoints):
        # Statistics variables
        self.collision_dismissals = 0
        self.vel_dismissals = 0
        self.acc_dismissals = 0
        
        startnode = self.wp_to_node(start_wp)
        startnode.parent = None
        startnode.parent_traj = Bezier([startnode.pos]*5, 1)
        startnode.cost = 0.0
        
        for wp in waypoints:
            goalnode = self.wp_to_node(wp)
            finalnode = self.plan_single_path(startnode, goalnode)
            # If planning fails, then fly what we got
            if not finalnode:
                finalnode = startnode
                break
            startnode = finalnode
        tot_path, tot_traj, tot_cost = self.retrace_path(finalnode)

        rospy.loginfo('RRT: Total path cost: {}'.format(tot_cost))
        rospy.loginfo('RRT: Collision dismissals: {}'.format(self.collision_dismissals))
        rospy.loginfo('RRT: Velocity dismissals: {}'.format(self.vel_dismissals))
        rospy.loginfo('RRT: Accelaration dismissals: {}'.format(self.acc_dismissals))

        # for bezier in tot_traj:
        #     rospy.loginfo('RRT: points: {}'.format(bezier.points))

        return tot_path, tot_traj

    def plan_single_path(self, startnode, goalnode):
        self.first_sample = True
        
        tree = RRTTree(startnode)

        #print('RRT: startpos = {}, startvel = {}'.format(startnode.pos, startnode.vel))
        #print('RRT: goalpos = {}, goalvel = {}'.format(goalnode.pos, goalnode.vel))

        for iter in xrange(1, self.iterations+1):

            rospy.loginfo_throttle(1, 'RRT: Iteration {}'.format(iter))

            randnode = self.sample_node(goalnode)
            nearnode = tree.find_nearest(randnode)
            newnodes = self.steer_bezier(nearnode, randnode)
            if len(newnodes) == 0:
                continue

            #print('RRT: randpos = {}, randvel = {}'.format(randnode.pos, randnode.vel))

            tree.add(newnodes)
            tree.rewire(newnodes, self.map)

            finalnode = self.reached_goal(newnodes, goalnode)
            if not finalnode is None:
                rospy.loginfo('RRT: Goal reached!')
                rospy.loginfo('RRT: {} total iterations'.format(iter))
                rospy.loginfo('RRT: {} nodes in tree'.format(len(tree)))
                return finalnode
        
        rospy.logwarn('RRT: Failed!')
        rospy.loginfo('RRT: {} total iterations'.format(iter))
        rospy.loginfo('RRT: {} nodes in tree'.format(len(tree)))

        publish_tree_to_rviz(tree)

        return 

    def sample_node(self, goal):
        # Always sample goal first time
        if self.first_sample:
            self.first_sample = False
            return goal
        elif np.random.rand() < self.goal_sample_prob:
            return goal
        else:
            # Sample position
            rand = np.random.rand(3)
            min = np.array(self.map.min)
            max = np.array(self.map.max)
            pos = Vec3(*(rand*max + (1-rand)*min))
            while self.map.query([pos]):
                rand = np.random.rand(3)
                min = np.array(self.map.min)
                max = np.array(self.map.max)
                pos = Vec3(*(rand*max + (1-rand)*min))

            # Sample velocity
            vel = Vec3(*(np.random.rand(3)))
            while abs(vel) > 1:
                vel = Vec3(*(np.random.rand(3)))
            vel *= self.vel_max
            
            # "Sample" yaw
            #yaw = np.random.rand() * 2*math.pi
            yaw = 0.0

            return RRTNode(pos, vel, yaw)

    def steer(self, parentnode, targetnode):
        start = parentnode.pos
        diff = targetnode.pos - start
        dist = abs(diff)
        direction = diff / dist
        step_len = min(dist, self.max_step_len)
        stop = start + direction * step_len

        line = Line(start, stop)
        if self.map.query(line):
            return []

        newnode = RRTNode(stop, targetnode.yaw)
        newnode.parent = parentnode
        newnode.cost = parentnode.cost + self.cost(parentnode, newnode)
        return [newnode]

    def steer_bezier(self, parentnode, targetnode):

        # Ensure target node is at most vel_max * max_step_time away
        diff = targetnode.pos - parentnode.pos
        dist = abs(diff)
        direction = diff / dist
        #step_len = min(dist, self.vel_max * self.max_step_time)
        step_len = min(dist, 1)
        stop = parentnode.pos + direction * step_len
        if self.map.query([stop]):
            return []
        targetnode = RRTNode(stop, targetnode.vel, targetnode.yaw)

        for T in np.linspace(self.min_step_time, self.max_step_time, num=11):
            bezier = newQuarticBezier(parentnode, targetnode, T)
            if self.check_bezier(bezier):
                targetnode.parent = parentnode
                targetnode.parent_traj = bezier
                targetnode.cost = parentnode.cost + self.cost_bezier(parentnode, targetnode)
                return [targetnode]
        
        return []

    def steer_bezier_stepwise(self, parentnode, targetnode):

        # Ensure target node is at most vel_max * max_step_time away
        diff = targetnode.pos - parentnode.pos
        dist = abs(diff)
        direction = diff / dist
        step_len = min(dist, self.vel_max * self.max_step_time)
        stop = parentnode.pos + direction * step_len
        targetnode = RRTNode(stop, targetnode.vel, targetnode.yaw)

        for T in np.linspace(self.min_step_time, self.max_step_time, num=5):
            bezier = newQuarticBezier(parentnode, targetnode, T)
            lower_bezier, bezier = bezier.split(self.min_step_time)

            newnodes = []
            
            while self.check_bezier(lower_bezier):
                newnode = RRTNode(lower_bezier.pos(lower_bezier.T), 
                                  lower_bezier.vel(lower_bezier.T), 
                                  targetnode.yaw)
                newnode.parent = parentnode
                newnode.parent_traj = lower_bezier
                newnode.cost = parentnode.cost + self.cost_bezier(parentnode, newnode)
                newnodes.append(newnode)

                if bezier.T > 0:
                    lower_bezier, bezier = bezier.split(min(self.min_step_time, bezier.T))
                else:
                    break
            
            if len(newnodes) > 1:
                print('RRT.steer(): {} nodes added at once.'.format(len(newnodes)))

            if len(newnodes) > 0:
                return newnodes


        return []

    def steer_bezier_binary(self, parentnode, targetnode):

        # Ensure target node is at most vel_max * max_step_time away
        diff = targetnode.pos - parentnode.pos
        dist = abs(diff)
        direction = diff / dist
        step_len = min(dist, self.vel_max * self.max_step_time / 10)
        stop = parentnode.pos + direction * step_len
        targetnode = RRTNode(stop, targetnode.yaw)

        # If kd-constraints are broken at max T, then it is not working anyway
        max_bezier = newQuarticBezier(parentnode, targetnode, self.max_step_time)
        if not self.check_kd_bezier(max_bezier):
            return []

        # If min is ok, then we are happy
        min_bezier = newQuarticBezier(parentnode, targetnode, self.min_step_time)
        if self.check_bezier(min_bezier):
            targetnode.parent = parentnode
            targetnode.parent_traj = min_bezier
            targetnode.cost = parentnode.cost + self.cost_bezier(parentnode, targetnode)
            return [targetnode]

        # Binary search for kinda optimal T
        T = self.binary_search(self.min_step_time, 
                               self.max_step_time, 
                               parentnode, 
                               targetnode)

        bezier = newQuarticBezier(parentnode, targetnode, T)
        targetnode.parent = parentnode
        targetnode.parent_traj = bezier
        targetnode.cost = parentnode.cost + self.cost_bezier(parentnode, targetnode)
        return [targetnode]

    def binary_search(self, min, max, parentnode, targetnode, tol=0.1):

        # Stop condition
        if max - min <= tol:
            return max
        
        # Recursive searching
        mid = (max + min) / 2
        bezier = newQuarticBezier(parentnode, targetnode, mid)
        if self.check_bezier(bezier):
            return self.binary_search(min, mid, parentnode, targetnode, tol)
        else:
            return self.binary_search(mid, max, parentnode, targetnode, tol)

    def check_bezier(self, bezier):
        """Check whether bezier curve obey geometric and kino-dynamic constraints."""

        # Collision
        if self.map.query(bezier):
            self.collision_dismissals += 1
            return False
        
        # Kino-dynamic
        return self.check_kd_bezier(bezier)

    def check_kd_bezier(self, bezier):
        """Check whether bezier curve obey kino-dynamic constraints."""

        # max_vel = max(abs(bezier.vel(t)) for t in np.linspace(0, bezier.T, 51))
        # max_acc = max(abs(bezier.acc(t)) for t in np.linspace(0, bezier.T, 51))
        # rospy.loginfo_throttle(1, 'RRT: bezier max vel = {}'.format(max_vel))
        # rospy.loginfo_throttle(1, 'RRT: bezier max acc = {}'.format(max_acc))
        
        for t in np.linspace(0, bezier.T, 21):
            # Velocity constraints
            if abs(bezier.vel(t)) > self.vel_max:
                self.vel_dismissals += 1
                return False
            # Acceleration constraints
            if abs(bezier.acc(t)) > self.acc_max:
                self.acc_dismissals += 1
                return False

        return True

    def reached_goal(self, nodes, goal):
        bestnode = None
        bestdist = float('inf')
        for node in nodes:
            posdist = abs(node.pos - goal.pos)
            veldist = abs(node.vel - goal.vel)
            # print('RRT.reached_goal(): posdist = {}, veldist = {}'.format(round(posdist, 5), round(veldist, 5)))
            # print('RRT.reached_goal(): node.pos = {}, node.vel = {}'.format(node.pos, node.vel))
            # print('RRT.reached_goal(): goal.pos = {}, goal.vel = {}'.format(goal.pos, goal.vel))
            if posdist <= self.wp_pos_tol and veldist <= self.wp_vel_tol:
                if posdist + veldist < bestdist:
                    bestdist = posdist + veldist
                    bestnode = node

        return bestnode

    def retrace_path(self, node):
        cost = node.cost
        Waypoint = namedtuple('Waypoint', ('x', 'y', 'z', 'yaw'))
        path = []
        traj = []
        while not node is None:
            path.append(Waypoint(node.pos.x, node.pos.y, node.pos.z, node.yaw))
            traj.append(node.parent_traj)
            node = node.parent
        path.reverse()
        traj.reverse()
        return path, traj, cost

    def path_to_traj(self, path):
        start = path[0]
        prev_p3 = Vec3(start.x, start.y, start.z)
        prev_p2 = prev_p3
        prev_p1 = prev_p3
        prev_T = 5
        beziers = []
        for next_wp in path[1:]:
            T = 5
            p0 = prev_p3
            p1 = (T/prev_T) * (prev_p3 - prev_p2) + p0
            p2 = (T/prev_T)**2 * (prev_p1 - 2*prev_p2 + prev_p3) + 2*p1 - p0
            p3 = Vec3(next_wp.x, next_wp.y, next_wp.z)
            beziers.append(QuarticBezier([p0, p1, p2, p3], T))
            prev_p1 = p1
            prev_p2 = p2
            prev_p3 = p3
            prev_T = T
        return beziers

    def wp_to_node(self, wp):
        return RRTNode(Vec3(wp.x, wp.y, wp.z), Vec3(wp.vx, wp.vy, wp.vz), wp.yaw)

    @staticmethod
    def cost(startnode, endnode):
        return abs(endnode.pos - startnode.pos)
    
    @staticmethod
    def cost_bezier(startnode, endnode):
        return endnode.parent_traj.T
    
    
class BiRRT(object):

    _seed = np.random.randint(1000, 9999)
    #_seed = 6176

    def __init__(self, map):
        self.map = map
        
        rospy.loginfo('RRT: Initialising with random seed {}'.format(self._seed))
        np.random.seed(self._seed)

        # Access rrt ros parameters
        self.iterations         = rospy.get_param(rospy.get_namespace() + 'rrt/iterations')
        self.goal_sample_prob   = rospy.get_param(rospy.get_namespace() + 'rrt/goal_sample_prob')
        self.connect_trees_prob = rospy.get_param(rospy.get_namespace() + 'rrt/connect_trees_prob')
        #self.max_step_len       = rospy.get_param(rospy.get_namespace() + 'rrt/max_step_length')
        self.min_step_time      = rospy.get_param(rospy.get_namespace() + 'rrt/min_step_time')
        self.max_step_time      = rospy.get_param(rospy.get_namespace() + 'rrt/max_step_time')
        self.wp_pos_tol         = rospy.get_param(rospy.get_namespace() + 'rrt/waypoint_position_tolerance')
        self.wp_vel_tol         = rospy.get_param(rospy.get_namespace() + 'rrt/waypoint_velocity_tolerance')
        self.vel_max            = rospy.get_param(rospy.get_namespace() + 'crazyflie/vel_max')
        self.acc_max            = rospy.get_param(rospy.get_namespace() + 'crazyflie/acc_max')

    def plan_path(self, start_wp, waypoints):
        # Statistics variables
        self.collision_dismissals = 0
        self.vel_dismissals = 0
        self.acc_dismissals = 0
        
        startnode = self.startnode_from_wp(start_wp)
        wp_nodes = [startnode]
        for wp in waypoints:
            goalnode = self.goalnode_from_wp(wp)
            finalnode = self.plan_single_path(startnode, goalnode)
            # If planning fails, then fly what we got
            if not finalnode:
                finalnode = startnode
                break
            wp_nodes.append(finalnode)
            startnode = finalnode
        #tot_path, tot_traj, tot_cost = self.retrace_path(finalnode)
        tot_path, tot_traj, tot_cost = self.postprocess_path(wp_nodes)

        rospy.loginfo('RRT: Total path cost: {}'.format(tot_cost))
        rospy.loginfo('RRT: Collision dismissals: {}'.format(self.collision_dismissals))
        rospy.loginfo('RRT: Velocity dismissals: {}'.format(self.vel_dismissals))
        rospy.loginfo('RRT: Accelaration dismissals: {}'.format(self.acc_dismissals))

        return tot_path, tot_traj

    def plan_single_path(self, startnode, goalnode):
        
        start_tree = RRTTree(startnode)
        goal_tree = RRTTree(goalnode)

        # Start with oneway sampling from startnode to goalnode
        newnode = self.steer(startnode, goalnode)
        if newnode == goalnode:
            rospy.loginfo('RRT: Goal reached at first try!')
            finalnode = self.find_finalnode(newnode, goalnode)
            return finalnode
        elif not newnode is None:
            start_tree.add(newnode)

        iter = 0
        while not rospy.is_shutdown() and iter <= self.iterations:
            iter += 1

            rospy.loginfo_throttle(1, 'RRT: Iteration {}'.format(iter))

            # Try connect trees directly with equal probability of either direction
            if np.random.rand() < self.connect_trees_prob:
                if np.random.rand() < 0.5:
                    goal_candidate = goal_tree.sample_node()
                    start_nearnode = start_tree.find_nearest(goal_candidate)
                    start_candidate = self.steer(start_nearnode, goal_candidate)
                    if not start_candidate is None:
                        start_tree.add(start_candidate)
                else:
                    start_candidate = start_tree.sample_node()
                    goal_nearnode = goal_tree.find_nearest(start_candidate)
                    goal_candidate = self.steer(goal_nearnode, start_candidate, reverse=True)
                    if not goal_candidate is None:
                        goal_tree.add(goal_candidate)

            # Sample new configuration and try connect to both trees
            else:
                samplenode = self.sample_configuration()

                # Try connecting to start tree
                start_nearnode = start_tree.find_nearest(samplenode)
                start_candidate = self.steer(start_nearnode, samplenode)
                if not start_candidate is None:
                    start_tree.add(start_candidate)

                # Try connecting to goal tree
                goal_nearnode = goal_tree.find_nearest(samplenode)
                goal_candidate = self.steer(goal_nearnode, samplenode, reverse=True)
                if not goal_candidate is None:
                    goal_tree.add(goal_candidate)

            # Check candidates from both trees actually are connected
            if not start_candidate is None and start_candidate == goal_candidate:
                rospy.loginfo('RRT: Goal reached!')
                rospy.loginfo('RRT: {} total iterations'.format(iter))
                rospy.loginfo('RRT: {} nodes in start tree'.format(len(start_tree)))
                rospy.loginfo('RRT: {} nodes in goal tree'.format(len(goal_tree)))
                finalnode = self.find_finalnode(start_candidate, goal_candidate)
                return finalnode
        
        rospy.logwarn('RRT: Failed!')
        rospy.loginfo('RRT: {} total iterations'.format(iter))
        rospy.loginfo('RRT: {} nodes in start tree'.format(len(start_tree)))
        rospy.loginfo('RRT: {} nodes in goal tree'.format(len(goal_tree)))

        publish_trees_to_rviz(start_tree, goal_tree)

        return 

    def sample_configuration(self):

        # Sample position
        rand = np.random.rand(3)
        min = np.array(self.map.min)
        max = np.array(self.map.max)
        pos = Vec3(*(rand*max + (1-rand)*min))
        while self.map.query([pos]):
            rand = np.random.rand(3)
            min = np.array(self.map.min)
            max = np.array(self.map.max)
            pos = Vec3(*(rand*max + (1-rand)*min))

        # Sample velocity
        vel = Vec3(*(np.random.rand(3)))
        while abs(vel) > 1:
            vel = Vec3(*(np.random.rand(3)))
        vel *= self.vel_max
        
        # "Sample" yaw
        #yaw = np.random.rand() * 2*math.pi
        yaw = 0.0

        return RRTNode(pos, vel, yaw)

    def steer(self, connectnode, targetnode, reverse=False):

        # Change distance of target 
        diff = targetnode.pos - connectnode.pos
        dist = abs(diff)
        direction = diff / dist
        #step_len = min(dist, self.vel_max * self.max_step_time)
        step_len = min(dist, 1)
        stop = connectnode.pos + direction * step_len
        if self.map.query([stop]):
            return
        targetnode = RRTNode(stop, targetnode.vel, targetnode.yaw)

        for T in np.linspace(self.min_step_time, self.max_step_time, num=11):
            if reverse:
                bezier = Bezier.newPenticBezier(
                        targetnode.pos,
                        targetnode.vel,
                        Vec3(0, 0, 0),
                        connectnode.pos,
                        connectnode.vel,
                        connectnode.acc,
                        T
                )
                if self.check_bezier(bezier):
                    RRTNode.link_nodes(targetnode, connectnode, bezier)
                    return targetnode
            else:
                bezier = Bezier.newPenticBezier(
                        connectnode.pos,
                        connectnode.vel,
                        connectnode.acc,
                        targetnode.pos,
                        targetnode.vel,
                        Vec3(0, 0, 0),
                        T
                )
                if self.check_bezier(bezier):
                    RRTNode.link_nodes(connectnode, targetnode, bezier)
                    return targetnode
        
        return

    def check_bezier(self, bezier):
        """Check whether bezier curve obey geometric and kino-dynamic constraints."""

        # Collision
        if self.map.query(bezier):
            self.collision_dismissals += 1
            return False
        
        # Kino-dynamic
        return self.check_kd_bezier(bezier)

    def check_kd_bezier(self, bezier):
        """Check whether bezier curve obey kino-dynamic constraints."""

        # max_vel = max(abs(bezier.vel(t)) for t in np.linspace(0, bezier.T, 51))
        # max_acc = max(abs(bezier.acc(t)) for t in np.linspace(0, bezier.T, 51))
        # rospy.loginfo_throttle(1, 'RRT: bezier max vel = {}'.format(max_vel))
        # rospy.loginfo_throttle(1, 'RRT: bezier max acc = {}'.format(max_acc))
        
        for t in np.linspace(0, bezier.T, 21):
            # Velocity constraints
            if abs(bezier.vel(t)) > self.vel_max:
                self.vel_dismissals += 1
                return False
            # Acceleration constraints
            if abs(bezier.acc(t)) > self.acc_max:
                self.acc_dismissals += 1
                return False

        return True

    def retrace_path(self, node):
        cost = node.cost
        path = []
        traj = []
        while not node is None:
            path.append(Waypoint(
                    node.x, node.y, node.z,
                    node.vx, node.vy, node.vz,
                    node.yaw
            ))
            traj.append(node.prev_traj)
            node = node.prev_node
        path.reverse()
        traj.reverse()
        return path, traj, cost

    def postprocess_path(self, wp_nodes):

        pre_cost = wp_nodes[-1].cost
        for startnode, endnode in zip(wp_nodes[:-1], wp_nodes[1:]):
            self.refine_trajectory(startnode, endnode)

        node = wp_nodes[-1]
        cost = node.cost
        path = []
        traj = []
        while not node is None:
            path.append(Waypoint(
                    node.x, node.y, node.z,
                    node.vx, node.vy, node.vz,
                    node.yaw
            ))
            traj.append(node.prev_traj)
            node = node.prev_node
        path.reverse()
        traj.reverse()

        rospy.loginfo('RRT: Postprocessing done. Decreased cost by {}'.format(round(pre_cost - cost, 4)))
        return path, traj, cost

    def refine_trajectory(self, startnode, endnode):

        def get_nodes(startnode, endnode, split=False):
            """Extract list nodes from .prev_node attribute chain. Optionally split long trajectories."""
            split_threshold = 2
            nodes = []
            node = endnode
            while not node is startnode:
                if not split or node.prev_traj.T < split_threshold:
                    nodes.append(node)
                    node = node.prev_node
                else:
                    lower_bezier, upper_bezier = node.prev_traj.split(node.prev_traj.T/2)

                    newnode = RRTNode(upper_bezier.pos(0), upper_bezier.vel(0), node.yaw)
                    RRTNode.link_nodes(node.prev_node, newnode, lower_bezier)
                    RRTNode.link_nodes(newnode, node, upper_bezier)

                    nodes.append(node)
                    nodes.append(newnode)
                    node = newnode.prev_node
            
            nodes.append(startnode)
            nodes.reverse()
            return nodes
        
        nodes = get_nodes(startnode, endnode)
        
        # Pair-wise optimisation (Should this be done? Equivalent principle as in steer. Higher order perhaps?)
        pre_cost = endnode.cost
        for node1, node2 in zip(nodes[:-1], nodes[1:]):
            current_T = node2.cost - node1.cost
            assert isclose(current_T, node2.prev_traj.T)
            best_bezier = None

            for T in np.linspace(current_T, current_T/2, 5)[1:]:
                bezier = Bezier.newPenticBezier(node1.pos, node1.vel, node1.acc, node2.pos, node2.vel, node2.acc, T)
                if self.check_bezier(bezier):
                    best_bezier = bezier
            
            if not best_bezier is None:
                RRTNode.link_nodes(node1, node2, best_bezier)

        post_cost = endnode.cost
        print('RRT: Pairwise saved {} seconds'.format(round(pre_cost - post_cost, 4)))

        # Iteratively skip one node optimisation
        node_skipped = True
        while node_skipped:
            node_skipped = False
            for start_i in (0, 1):
                nodes = get_nodes(startnode, endnode, split=False)

                i = start_i
                while i < len(nodes) - 2:
                    node1 = nodes[i]
                    node2 = nodes[i+2]

                    current_T = node2.cost - node1.cost
                    best_bezier = None

                    for T in np.linspace(current_T, current_T/8, 11)[1:]:
                        bezier = Bezier.newPenticBezier(node1.pos, node1.vel, node1.acc, node2.pos, node2.vel, node2.acc, T)
                        if self.check_bezier(bezier):
                            best_bezier = bezier
                    
                    if best_bezier is None:
                        i += 1
                    else:
                        RRTNode.link_nodes(node1, node2, best_bezier)
                        print('Skipped node! dT = {}'.format(round(current_T - T, 4)))
                        node_skipped = True
                        i += 2

        return

    @staticmethod
    def find_finalnode(backwards_node, forwards_node):
        # Dangerous if nodes not equal
        assert backwards_node == forwards_node

        # Connect the two branches
        if forwards_node.next_node is None:
            return backwards_node
        else:
            RRTNode.link_nodes(backwards_node, forwards_node.next_node, forwards_node.next_traj)

            # Iterate forward until end if chain
            node = backwards_node
            while not node.next_node is None:
                node = node.next_node

            return node

    @staticmethod
    def startnode_from_wp(wp):
        pos = Vec3(wp.x, wp.y, wp.z)
        vel = Vec3(wp.vx, wp.vy, wp.vz)
        acc = Vec3(0, 0, 0)

        T = 1
        p2 = pos
        p1 = -T/2 * vel + p2
        p0 = T**2/2 * acc + 2*p1 - p2
        bezier = Bezier([p0, p1, p2], T)

        startnode = RRTNode(pos, vel, wp.yaw)
        startnode.set_prev_link(None, bezier)

        return startnode

    @staticmethod
    def goalnode_from_wp(wp):
        pos = Vec3(wp.x, wp.y, wp.z)
        vel = Vec3(wp.vx, wp.vy, wp.vz)
        acc = Vec3(0, 0, 0)

        T = 1
        p0 = pos
        p1 = T/2 * vel + p0
        p2 = T**2/2 * acc + 2*p1 - p0
        bezier = Bezier([p0, p1, p2], T)

        goalnode = RRTNode(pos, vel, wp.yaw)
        goalnode.set_next_link(None, bezier)

        return goalnode



class RRTTree(object):

    def __init__(self, root):
        self.root = root
        self.node_storage = NearestNeighbourGrid([root], bin_size=0.5)

    def __len__(self):
        return len(self.node_storage)

    def add(self, newnodes):
        if not isinstance(newnodes, list):
            newnodes = [newnodes]
        self.node_storage.add(newnodes)

    def find_nearest(self, node):
        nearests = self.node_storage.query_nn(node, 1)
        return nearests[0]

    def sample_node(self):
        return self.node_storage.sample_node()

    def rewire(self, newnode, map):
        # TODO: figure out how this works with bezier trajectories
        pass
        # nearests = self.node_storage.query_nn(newnode, 5)
        # for neighbour in nearests:
        #     if neighbour == newnode.parent:
        #         continue
        #     newcost = newnode.cost + RRT.cost(newnode, neighbour)
        #     if neighbour.cost > newcost:
        #         line = Line(neighbour.pos, newnode.pos)
        #         if map.query(line):
        #             neighbour.parent = newnode
        #             neighbour.cost = newcost


class NearestNeighbourGrid(object):

    def __init__(self, nodes, bin_size):
        self.bin_size = bin_size
        self.grid = defaultdict(lambda: [])

        self.add(nodes)

    def __len__(self):
        size = 0
        for list in self.grid.itervalues():
            size += len(list)
        return size

    def add(self, nodes):
        for node in nodes:
            index = self.exact_to_index(*node.pos)
            self.grid[index].append(node)

    def sample_node(self):
        samplenode = None
        buckets = self.grid.values()
        nrbuckets = len(buckets)
        while samplenode is None:
            i = np.random.randint(nrbuckets)
            bucket = buckets[i]
            if len(bucket) > 0:
                samplenode = np.random.choice(bucket)
        return samplenode

    def query_nn(self, node, k):
        index = self.exact_to_index(*node.pos)
        all_idxs = {(index[0]+dx, index[1]+dy, index[2]+dz) for dx in (-1,0,1) for dy in (-1,0,1) for dz in (-1,0,1)}
        neighbours = []
        for idx in all_idxs:
            neighbours.extend(self.grid[idx])
        neighbours.sort(key=lambda n: abs(node.pos - n.pos))
        if not neighbours:
            return [self.query_nn_brute(node)]
        elif len(neighbours) <= k:
            return neighbours
        else:
            return neighbours[:k]

    def query_nn_brute(self, node):
        neighbours = []
        for local_neighbours in self.grid.itervalues():
            neighbours.extend(local_neighbours)
        min_dist = float('inf')
        min_neighbour = None
        for neighbour in neighbours:
            dist = abs(neighbour.pos - node.pos)
            if dist <= min_dist:
                min_dist = dist
                min_neighbour = neighbour
        return min_neighbour

    def exact_to_index(self, x, y, z):
        return int(math.floor(x / self.bin_size)), int(math.floor(y / self.bin_size)), int(math.floor(z / self.bin_size))


class RRTNode(object):

    def __init__(self, pos, vel, yaw=0.0):
        self._pos = pos
        self._vel = vel
        self._acc = None
        self._yaw = yaw 

    def __repr__(self):
        return 'RRTNode(pos: ({}, {}, {}), vel: ({}, {}, {}), yaw: {})'.format(
                round(self.x, 4), 
                round(self.y, 4), 
                round(self.z, 4), 
                round(self.vx, 4), 
                round(self.vy, 4), 
                round(self.vz, 4), 
                round(self.yaw, 4))

    def __nonzero__(self):
        return True

    def __eq__(self, other):
        if isinstance(other, RRTNode):
            if not self._acc is None and not other._acc is None:
                return (    self._pos == other._pos 
                        and self._vel == other._vel 
                        and self._acc == other._acc 
                        and isclose(self._yaw, other._yaw))
            else:
                return (    self._pos == other._pos 
                        and self._vel == other._vel 
                        and isclose(self._yaw, other._yaw))
        else:
            return False
    def __ne__(self, other):
        if isinstance(other, RRTNode):
            if not self._acc is None and not other._acc is None:
                return (   self._pos != other._pos 
                        or self._vel != other._vel 
                        or self._acc != other._acc 
                        or isclose(self._yaw, other._yaw))
            else:
                return (   self._pos != other._pos 
                        or self._vel != other._vel 
                        or isclose(self._yaw, other._yaw))
        else:
            return True

    @property
    def pos(self):
        return self._pos
    @property
    def x(self):
        return self._pos.x
    @property
    def y(self):
        return self._pos.y
    @property
    def z(self):
        return self._pos.z

    @property
    def vel(self):
        return self._vel
    @property
    def vx(self):
        return self._vel.x
    @property
    def vy(self):
        return self._vel.y
    @property
    def vz(self):
        return self._vel.z

    @property
    def acc(self):
        return self._acc
    
    @property
    def yaw(self):
        return self._yaw

    @property
    def next_node(self):
        return self._next_node
    @property
    def next_traj(self):
        return self._next_traj
    @property
    def prev_node(self):
        return self._prev_node
    @property
    def prev_traj(self):
        return self._prev_traj

    @property
    def cost(self):
        return self._cost
    @cost.setter
    def cost(self, newcost):
        self._cost = newcost
        try:
            self._next_node.cost = self._cost + self._next_traj.T
        except AttributeError:
            pass
    @property
    def cost_to_goal(self):
        return self._cost_to_goal
    @cost_to_goal.setter
    def cost_to_goal(self, newcost):
        self._cost_to_goal = newcost
        try:
            self._prev_node.cost_to_goal = self._cost_to_goal + self._prev_traj.T
        except AttributeError:
            pass

    def set_next_link(self, next_node, next_traj):
        assert self._pos == next_traj.pos(0)
        assert self._vel == next_traj.vel(0)
        if self._acc is None:
            self._acc = next_traj.acc(0)
        else:
            assert self._acc == next_traj.acc(0)

        self._next_node = next_node
        self._next_traj = next_traj

        if next_node is None:
            self.cost_to_goal = 0
        else:
            try:
                self.cost_to_goal = next_node.cost_to_goal + next_traj.T
            except AttributeError:
                pass

    def set_prev_link(self, prev_node, prev_traj):
        assert self._pos == prev_traj.pos(-1)
        assert self._vel == prev_traj.vel(-1)
        if self._acc is None:
            self._acc = prev_traj.acc(0)
        else:
            assert self._acc == prev_traj.acc(0)

        self._prev_node = prev_node
        self._prev_traj = prev_traj

        if prev_node is None:
            self.cost = 0
        else:
            try:
                self.cost = prev_node.cost + prev_traj.T
            except AttributeError:
                pass

    @staticmethod
    def link_nodes(node1, node2, traj):
        node1.set_next_link(node2, traj)
        node2.set_prev_link(node1, traj)


def newCubicBezier(parentnode, targetnode, T):
    parent_p1 = parentnode.parent_traj.points[1]
    parent_p2 = parentnode.parent_traj.points[2]
    parent_p3 = parentnode.parent_traj.points[3]
    parent_T = parentnode.parent_traj.T

    p0 = parent_p3
    p1 = (T/parent_T) * (parent_p3 - parent_p2) + p0
    p2 = (T/parent_T)**2 * (parent_p1 - 2*parent_p2 + parent_p3) + 2*p1 - p0
    p3 = targetnode.pos

    return Bezier([p0, p1, p2, p3], T)

def newQuarticBezier(connectnode, targetnode, T, reverse=False):
    if reverse:
        end_acc = connectnode.child_traj.acc(0)

        p4 = connectnode.pos
        p3 = -T/4 * connectnode.vel + p4
        p2 = T**2/12 * end_acc + 2*p3 - p4
        p0 = targetnode.pos
        p1 = T/4 * targetnode.vel + p0

        # Ad-hoc fix?
        p2 = Vec3(p2.x, p2.y, (p0.z + p4.z) / 2)
    else:
        start_acc = connectnode.parent_traj.acc(connectnode.parent_traj.T)

        p0 = connectnode.pos
        p1 = T/4 * connectnode.vel + p0
        p2 = T**2/12 * start_acc + 2*p1 - p0
        p4 = targetnode.pos
        p3 = -T/4 * targetnode.vel + p4

        # Ad-hoc fix?
        p2 = Vec3(p2.x, p2.y, (p0.z + p4.z) / 2)

    return Bezier([p0, p1, p2, p3, p4], T)

def publish_tree_to_rviz(tree):
    rviz_marker_pub = rospy.Publisher('/tree_marker', Marker, queue_size=3)

    # Does this delete all markers on all marker topics or only this topic?
    # Clear all previous rviz markers
    delete_msg = Marker()
    delete_msg.action = Marker.DELETEALL
    rviz_marker_pub.publish(delete_msg)

    rospy.sleep(0.3)

    sphere_marker, line_marker = tree_to_markers(tree)

    rospy.sleep(0.3)

    rviz_marker_pub.publish(sphere_marker)
    rviz_marker_pub.publish(line_marker)

    return

def publish_trees_to_rviz(start_tree, goal_tree):
    rviz_marker_pub = rospy.Publisher('/tree_marker', Marker, queue_size=5)

    # Does this delete all markers on all marker topics or only this topic?
    # Clear all previous rviz markers
    delete_msg = Marker()
    delete_msg.action = Marker.DELETEALL
    rviz_marker_pub.publish(delete_msg)

    rospy.sleep(0.3)

    start_sphere_marker, start_line_marker = tree_to_markers(start_tree, ns='start_')
    goal_sphere_marker, goal_line_marker = tree_to_markers(goal_tree, ns='goal_')

    rospy.sleep(0.3)

    rviz_marker_pub.publish(start_sphere_marker)
    rviz_marker_pub.publish(start_line_marker)
    rviz_marker_pub.publish(goal_sphere_marker)
    rviz_marker_pub.publish(goal_line_marker)

    return

def tree_to_markers(tree, ns=''):

    sphere_marker = Marker()
    sphere_marker.header.frame_id = 'map'
    sphere_marker.ns = ns + 'tree_nodes'
    sphere_marker.id = 10001
    sphere_marker.type = Marker.SPHERE_LIST
    sphere_marker.action = Marker.ADD
    sphere_marker.color.r = 0.0
    sphere_marker.color.g = 0.8
    sphere_marker.color.b = 0.8
    sphere_marker.color.a = 1.0
    sphere_marker.scale.x = 0.02
    sphere_marker.scale.y = 0.02
    sphere_marker.scale.z = 0.02
    sphere_marker.points = []

    line_marker = Marker()
    line_marker.header.frame_id = 'map'
    line_marker.ns = ns + 'tree_connections'
    line_marker.id = 10002
    line_marker.type = Marker.LINE_LIST
    line_marker.action = Marker.ADD
    line_marker.color.r = 0.0
    line_marker.color.g = 0.8
    line_marker.color.b = 0.8
    line_marker.color.a = 1.0
    line_marker.scale.x = 0.005
    line_marker.scale.y = 0.005
    line_marker.scale.y = 0.005
    line_marker.points = []

    for square in tree.node_storage.grid.itervalues():
        for node in square:
            sphere_marker.points.append(Point(node.x, node.y, node.z))
            if node == tree.root:
                continue
            elif ns == 'goal_':
                connector = node.child
            else:
                connector = node.parent
            line_marker.points.append(Point(node.x, node.y, node.z))
            line_marker.points.append(Point(connector.x, connector.y, connector.z))

    return sphere_marker, line_marker
