#!/usr/bin/env python

from __future__ import division

import math
import json
import copy
import Queue
from collections import namedtuple
from itertools import count

import rospy
import tf2_ros
import tf2_geometry_msgs
from nav_msgs.msg import Path
from geometry_msgs.msg import PoseStamped, Point
from visualization_msgs.msg import Marker, MarkerArray
from tf.transformations import quaternion_from_euler

from octomap import create_map, Vec3
from planning import HeapPriorityQueue, Node

def transform_to_pose(tf):
    pose = PoseStamped()
    pose.header = tf.header
    pose.pose.position.x = tf.transform.translation.x
    pose.pose.position.y = tf.transform.translation.y
    pose.pose.position.z = tf.transform.translation.z
    pose.pose.orientation = tf.transform.rotation
    return pose

def newPoseStamped(x, y, z, roll, pitch, yaw, frame_id, stamp=None):
    pose = PoseStamped()
    pose.header.frame_id = frame_id
    pose.pose.position = Point(x=x, y=y, z=z)
    (pose.pose.orientation.x,
     pose.pose.orientation.y,
     pose.pose.orientation.z,
     pose.pose.orientation.w) = quaternion_from_euler(roll, pitch, yaw)

    if stamp is None:
        pose.header.stamp = rospy.Time.now()
    else:
        pose.header.stamp = stamp

    return pose


class GlobalPlanner(object):

    def __init__(self):
        rospy.init_node('global_planner', log_level=rospy.INFO)
        rospy.loginfo(rospy.get_name() + ': Initialising node...')

        # Access ros parameters
        use_rviz            = rospy.get_param(rospy.get_name() + '/use_rviz')
        map_file            = rospy.get_param(rospy.get_name() + '/map_file')
        collision_radius    = rospy.get_param(rospy.get_name() + '/collision_radius')
        tfprefix            = rospy.get_param(rospy.get_name() + '/tfprefix')
        trajectory_topic    = rospy.get_param(rospy.get_name() + '/trajectory_topic')

        # Publishers
        self.traj_pub = rospy.Publisher(trajectory_topic, Path, queue_size=1)

        # Set up tf stuff
        if tfprefix:
            self.base_frame = tfprefix + '/base_link'
        else:
            self.base_frame = 'base_link'
        self.tf_buff = tf2_ros.Buffer()
        self.tf_lstn = tf2_ros.TransformListener(self.tf_buff)

        # Parse world map
        rospy.loginfo(rospy.get_name() + ': Creating octomap...')
        with open(map_file, 'r') as file:
            data = json.load(file)
        self.map, points = create_map(data, collision_radius)
        rospy.loginfo(rospy.get_name() + ': Octomap created.')

        # Publish map shapes to rviz
        if use_rviz:
            publish_map_to_rviz(self.map, points)

        return



    def start(self):
        # TODO: Add while not shutdown loop for online replanning

        # Creates PoseStamped at every waypoint (inefficient legacy code)
        start_pose = newPoseStamped(0, 0, 0, 0, 0, 0, self.base_frame)
        start_pose = self.tf_buff.transform(start_pose, 'map', rospy.Duration(0.2))
        start_pose.pose.position.z = 0.4
        poses = [start_pose]
        for gate in self.map.gates:
            poses += self.get_gate_poses(gate)

        # Convert to actual waypoints
        Waypoint = namedtuple('Waypoint', ('x', 'y'))
        waypoints = [Waypoint(pose.pose.position.x, pose.pose.position.y) for pose in poses]

        # Run A*
        path_2d = self.a_star(waypoints[0], waypoints[1:], self.map)

        # Convert path of nodes to nav_msgs/Path
        nav_poses = []
        for node in path_2d:
            x, y = self.index_to_exact(node.xi, node.yi)
            pose = newPoseStamped(x, y, 0.4, 0, 0, 0, 'map')
            nav_poses.append(pose)

        path = Path()
        path.header.frame_id = 'map'
        path.header.stamp = rospy.Time.now()
        path.poses = nav_poses

        self.traj_pub.publish(path)

    def get_gate_poses(self, gate):
        """Gets one pose before and one pose after gate."""
        theta = gate['heading'] * 2*math.pi/360
        normal = Vec3(math.cos(theta), math.sin(theta), 0.0)

        point1 = gate['position'] - 0.2*normal
        point2 = gate['position'] + 0.2*normal

        pose1 = PoseStamped()
        pose1.header.frame_id = 'map'
        (pose1.pose.position.x,
         pose1.pose.position.y,
         pose1.pose.position.z) = point1.x, point1.y, point1.z
        (pose1.pose.orientation.x,
         pose1.pose.orientation.y,
         pose1.pose.orientation.z,
         pose1.pose.orientation.w) = quaternion_from_euler(0.0, 0.0, theta)

        pose2 = PoseStamped()
        pose2.header.frame_id = 'map'
        (pose2.pose.position.x,
         pose2.pose.position.y,
         pose2.pose.position.z) = point2.x, point2.y, point2.z
        (pose2.pose.orientation.x,
         pose2.pose.orientation.y,
         pose2.pose.orientation.z,
         pose2.pose.orientation.w) = quaternion_from_euler(0.0, 0.0, theta)

        return [pose1, pose2]

    def a_star(self, start, waypoints, map):
        rospy.loginfo(rospy.get_name() + ': Running A*...')

        self.grid_size = 0.05

        # Dummy return variable
        path = []

        # Create start node
        xi, yi = self.exact_to_index(*start)
        startnode = Node(xi, yi, wpi=0)
        startnode.parent = None
        startnode.cost = 0
        startnode.heuristic = self.heuristic(startnode, waypoints)

        # A* loop setup
        openset = HeapPriorityQueue()
        openset.push(startnode)
        closedset = set()
        iter = 0

        # A* main loop
        while not openset.is_empty():
            currentnode = openset.pop()
            closedset.add(currentnode.key)

            iter += 1
            rospy.loginfo_throttle(1, rospy.get_name() + ': A* iteration {}'.format(iter))

            if self.reached_goal(currentnode, waypoints):
                path = self.retrace_path(currentnode)
                break
            else:
                self.update_neighbours(currentnode, waypoints, map, openset, closedset)

        if path:
            rospy.loginfo(rospy.get_name() + ': A* succeded!')
        else:
            rospy.logwarn(rospy.get_name() + ': A* failed!')
        return path

    def update_neighbours(self, currentnode, waypoints, map, openset, closedset):

        neighbours = {(dx,dy) for dx in (-1,0,1) for dy in (-1,0,1) if (dx or dy)}

        for dx, dy in neighbours:
            newnode = self.get_newnode(currentnode, dx, dy, waypoints)

            x, y = self.index_to_exact(newnode.xi, newnode.yi)
            position = Vec3(x, y, 0.4)

            if newnode.key in closedset:
                continue
            elif map.query([position]):
                continue
            elif newnode.key in openset:
                old_prio = openset.get_item(newnode.key).prio
                if newnode.prio <= old_prio:
                    openset.update_queue(newnode)
            else:
                openset.push(newnode)

    def get_newnode(self, parent, dx, dy, waypoints):
        newnode = Node(parent.xi+dx, parent.yi+dy, parent.wpi)
        if self.reached_wp(newnode, waypoints[parent.wpi]):
            newnode = Node(parent.xi+dx, parent.yi+dy, parent.wpi+1)

        newnode.parent = parent
        newnode.cost = parent.cost + (dx**2 + dy**2)**0.5 * self.grid_size
        newnode.heuristic = self.heuristic(newnode, waypoints)
        return newnode

    def retrace_path(self, node):
        path = []
        while not node is None:
            path.append(node)
            node = node.parent
        return reversed(path)

    def reached_goal(self, node, waypoints):
        return node.wpi == len(waypoints)

    def reached_wp(self, node, waypoint):
        nx, ny = self.index_to_exact(node.xi, node.yi)
        wpx, wpy = waypoint.x, waypoint.y
        return ((nx-wpx)**2 + (ny-wpy)**2)**0.5 <= 0.1

    def heuristic(self, node, waypoints):
        heuristic = 0
        x1, y1 = self.index_to_exact(node.xi, node.yi)
        for wp in waypoints[node.wpi:]:
            x2, y2 = wp.x, wp.y
            heuristic += ((x1-x2)**2 + (y1-y2)**2)**0.5
            x1, y1 = x2, y2
        return heuristic

    def exact_to_index(self, x, y):
        return int(math.floor(x / self.grid_size)), int(math.floor(y / self.grid_size))

    def index_to_exact(self, xi, yi):
        return xi * self.grid_size, yi * self.grid_size


def publish_map_to_rviz(map, points=[]):
    rviz_marker_pub = rospy.Publisher('/visualization_marker_array', MarkerArray, queue_size=2)

    # Clear all previous rviz markers
    delete_msg = MarkerArray()
    delete_marker = Marker()
    delete_marker.action = Marker.DELETEALL
    delete_msg.markers = [delete_marker]
    rviz_marker_pub.publish(delete_msg)

    occu_size2cube_map = {}
    free_size2cube_map = {}
    queue = Queue.Queue()
    queue.put(map.octree)
    while not queue.empty():
        tree = queue.get()
        if tree.isoccupied:
            if not map.airspace_min < tree.center < map.airspace_max:
                continue
            if not tree.size in occu_size2cube_map:
                occu_size2cube_map[tree.size] = set()
            occu_size2cube_map[tree.size].add(tree.center)
        elif tree.isfree:
            if not map.airspace_min < tree.center < map.airspace_max:
                continue
            if not tree.size in free_size2cube_map:
                free_size2cube_map[tree.size] = set()
            free_size2cube_map[tree.size].add(tree.center)
        else:
            for subtree in tree.subtrees:
                queue.put(subtree)

    id_gen = count()
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
    markers.append(marker)

    marker_array = MarkerArray()
    marker_array.markers = markers

    rviz_marker_pub.publish(marker_array)

    return



if __name__ == '__main__':
    planner = GlobalPlanner()
    planner.start()

    rospy.spin()




