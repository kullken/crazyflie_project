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

from geometry import Vec3
from kdmap import create_kdmap, Kdmap
from octomap import create_octomap, Octomap
from rrt import RRT
from a_star import AStar

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
        tfprefix            = rospy.get_param(rospy.get_name() + '/tfprefix')
        trajectory_topic    = rospy.get_param(rospy.get_name() + '/trajectory_topic')
        collision_radius    = rospy.get_param(rospy.get_namespace() + 'map/collision_radius')

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
        rospy.loginfo(rospy.get_name() + ': Creating map...')
        with open(map_file, 'r') as file:
            data = json.load(file)
        self.map, points = create_octomap(data, collision_radius)
        rospy.loginfo(rospy.get_name() + ': Map created.')

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
        Waypoint = namedtuple('Waypoint', ('x', 'y', 'z', 'yaw'))
        waypoints = [Waypoint(p.pose.position.x, p.pose.position.y, p.pose.position.z, 0.0) for p in poses]

        # Run path planning
        #planner = RRT(self.map)
        planner = AStar(self.map)
        path = planner.plan_path(waypoints[0], waypoints[1:])

        # Convert path of nodes to nav_msgs/Path
        nav_poses = []
        for wp in path:
            pose = newPoseStamped(wp.x, wp.y, wp.z, 0, 0, wp.yaw, 'map')
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


def publish_map_to_rviz(map, points):
    rviz_marker_pub = rospy.Publisher('/map_marker_array', MarkerArray, queue_size=2)

    # Clear all previous rviz markers
    delete_msg = MarkerArray()
    delete_marker = Marker()
    delete_marker.action = Marker.DELETEALL
    delete_msg.markers = [delete_marker]
    rviz_marker_pub.publish(delete_msg)

    rospy.sleep(0.5)

    id_gen = count()

    if type(map) == Kdmap:
        markers = kdmap_to_msgs(map, id_gen)
    else:
        markers = octomap_to_msgs(map, id_gen)

    markers.append(points_to_msg(points, id_gen))

    marker_array = MarkerArray()
    marker_array.markers = markers

    #rospy.loginfo(rospy.get_name() + ': Publishing to rviz...\n{}'.format(marker_array))
    rospy.sleep(0.5)

    rviz_marker_pub.publish(marker_array)

    return

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



if __name__ == '__main__':
    planner = GlobalPlanner()
    planner.start()

    rospy.spin()




