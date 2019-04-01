#!/usr/bin/env python

from __future__ import division

import math
import json
import numpy as np

import rospy
import tf2_ros
import tf2_geometry_msgs
from nav_msgs.msg import Path
from geometry_msgs.msg import PoseStamped, Point
from visualization_msgs.msg import Marker, MarkerArray
from tf.transformations import quaternion_from_euler

from crazyflie_driver.msg import TrajectoryPolynomialPiece
from dd241909_msgs.msg import Trajectory

from geometry import Vec3, Waypoint
from maps import create_map
from rrt import BiRRT as RRT

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
        map_file            = rospy.get_param(rospy.get_name() + '/map_file')
        tfprefix            = rospy.get_param(rospy.get_name() + '/tfprefix')
        path_topic          = rospy.get_param(rospy.get_name() + '/path_topic')
        trajectory_topic    = rospy.get_param(rospy.get_name() + '/trajectory_topic')
        self.use_rviz       = rospy.get_param(rospy.get_name() + '/use_rviz')
        self.wp_gate_dist   = rospy.get_param(rospy.get_namespace() + 'rrt/waypoint_gate_distance')
        self.wp_gate_vel    = rospy.get_param(rospy.get_namespace() + 'rrt/waypoint_gate_velocity')

        # Publishers
        self.path_pub = rospy.Publisher(path_topic, Path, queue_size=1)
        self.traj_pub = rospy.Publisher(trajectory_topic, Trajectory, queue_size=1)

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
        self.map = create_map(data, type='grid')
        rospy.loginfo(rospy.get_name() + ': Map created.')

        return

    def start(self):
        # TODO: Add while not shutdown loop for online replanning

        # Create waypoints for the planner to pass through
        waypoints = self.create_waypoints(self.map.gates)

        # Run path planning
        planner = RRT(self.map)
        path, traj = planner.plan_path(waypoints[0], waypoints[1:])

        # Convert path of nodes to nav_msgs/Path
        nav_poses = []
        for wp in path:
            pose = newPoseStamped(wp.x, wp.y, wp.z, 0, 0, wp.yaw, 'map')
            nav_poses.append(pose)
        path_msg = Path()
        path_msg.header.frame_id = 'map'
        path_msg.header.stamp = rospy.Time.now()
        path_msg.poses = nav_poses

        # Convert trajectory of Beziers to dd241909_msgs/Trajectory
        traj_pieces = []
        for bezier in traj:
            piece = TrajectoryPolynomialPiece()
            for c in bezier.coeffs:
                piece.poly_x.append(c.x)
                piece.poly_y.append(c.y)
                piece.poly_z.append(c.z)
                piece.poly_yaw.append(0.0)
            piece.duration = rospy.Duration(bezier.T)
            traj_pieces.append(piece)
        traj_msg = Trajectory()
        traj_msg.header.frame_id = 'map'
        traj_msg.header.stamp = rospy.Time.now()
        traj_msg.pieces = traj_pieces

        # Publish path and trajectory
        self.path_pub.publish(path_msg)
        self.traj_pub.publish(traj_msg)

        # Visualise trajectory in rviz
        if self.use_rviz:
            publish_traj_to_rviz(traj_msg)

    def create_waypoints(self, gates):
        # Get start pose from tf tree
        start_pose = newPoseStamped(0, 0, 0, 0, 0, 0, self.base_frame)
        start_pose = self.tf_buff.transform(start_pose, 'map', rospy.Duration(0.2))

        # Start creating waypoints
        start_wp = Waypoint(
                start_pose.pose.position.x,
                start_pose.pose.position.y,
                0.1, 0.0, 0.0, 0.0, 0.0
        )
        takeoff_wp = Waypoint(
                start_pose.pose.position.x,
                start_pose.pose.position.y,
                0.4, 0.0, 0.0, 0.0, 0.0
        )

        # Waypoints in front of and after each gate
        waypoints = [start_wp, takeoff_wp]
        for gate in self.map.gates:
            before_wp, after_wp = self.gate_to_waypoints(gate)
            waypoints.extend([before_wp, after_wp])

        # Last waypoint 0.5 m after last gate
        theta = self.map.gates[-1]['heading'] * 2*math.pi/360
        normal = Vec3(math.cos(theta), math.sin(theta), 0.0)
        pos = self.map.gates[-1]['position'] + 0.5*normal
        vel = self.wp_gate_vel * normal
        last_wp = Waypoint(pos.x, pos.y, pos.z, vel.x, vel.y, vel.z, 0.0)
        waypoints.append(last_wp)

        return waypoints

    def gate_to_waypoints(self, gate):
        theta = gate['heading'] * 2*math.pi/360
        normal = Vec3(math.cos(theta), math.sin(theta), 0.0)

        pos1 = gate['position'] - self.wp_gate_dist*normal
        pos2 = gate['position'] + self.wp_gate_dist*normal

        vel1 = self.wp_gate_vel * normal
        vel2 = self.wp_gate_vel * normal

        wp1 = Waypoint(pos1.x, pos1.y, pos1.z, vel1.x, vel1.y, vel1.z, 0.0)
        wp2 = Waypoint(pos2.x, pos2.y, pos2.z, vel2.x, vel2.y, vel2.z, 0.0)

        return wp1, wp2

    def get_gate_poses(self, gate):
        """Gets one pose before and one pose after gate."""
        theta = gate['heading'] * 2*math.pi/360
        normal = Vec3(math.cos(theta), math.sin(theta), 0.0)

        point1 = gate['position'] - self.wp_gate_dist*normal
        point2 = gate['position'] + self.wp_gate_dist*normal

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


def publish_traj_to_rviz(traj_msg):
    rviz_marker_pub = rospy.Publisher('/trajectory_marker', Marker, queue_size=2)

    # Does this delete all markers on all marker topics or only this topic?
    # Clear all previous rviz markers
    delete_msg = Marker()
    delete_msg.action = Marker.DELETEALL
    rviz_marker_pub.publish(delete_msg)

    rospy.sleep(0.3)

    marker = Marker()
    marker.header.frame_id = 'map'
    marker.ns = 'trajectory_points'
    marker.id = 10001
    marker.type = Marker.SPHERE_LIST
    marker.action = Marker.ADD
    marker.color.r = 0.8
    marker.color.g = 0.0
    marker.color.b = 0.8
    marker.color.a = 1.0
    marker.scale.x = 0.02
    marker.scale.y = 0.02
    marker.scale.z = 0.02
    marker.points = []

    dt = 0.2
    tot_t = 0
    accum_t = 0
    for piece in traj_msg.pieces:
        nrterms = len(piece.poly_x)
        coeffs = np.array([piece.poly_x,
                           piece.poly_y,
                           piece.poly_z])
        
        while tot_t - accum_t < piece.duration.to_sec():
            t = tot_t - accum_t
            tot_t += dt

            tv = np.power([t]*nrterms, range(nrterms))
            pos = np.dot(coeffs, tv)
            marker.points.append(Point(x=pos[0], y=pos[1], z=pos[2]))

        accum_t += piece.duration.to_sec()
    
    rospy.sleep(0.3)

    rviz_marker_pub.publish(marker)

    return


if __name__ == '__main__':
    planner = GlobalPlanner()
    planner.start()

    rospy.spin()




