#!/usr/bin/env python

from __future__ import division

import math
import json
import numpy as np

import rospy
import tf2_ros
import tf2_geometry_msgs
import actionlib
from nav_msgs.msg import Path
from geometry_msgs.msg import PoseStamped, Point
from visualization_msgs.msg import Marker, MarkerArray
from tf.transformations import quaternion_from_euler, euler_from_quaternion

from crazyflie_driver.msg import TrajectoryPolynomialPiece

from dd241909_msgs.msg import Trajectory
from dd241909_msgs.msg import SimpleAction, SimpleResult

from maps import create_map
from rrt import RRT

#from geometry import Vec3, Waypoint

import pyximport
pyximport.install()
from cgeometry import Vec3, Waypoint

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

        self.wait_rate = rospy.Rate(10)

        # Access ros parameters
        map_file            = rospy.get_param('~map_file')
        tfprefix            = rospy.get_param('~tfprefix')
        trajectory_topic    = rospy.get_param('~trajectory_topic')
        plan_action         = rospy.get_param('~plan_action')
        self.use_rviz       = rospy.get_param('~use_rviz')
        self.wp_pre_gate_dist   = rospy.get_param(rospy.get_namespace() + 'rrt/waypoint_pre_gate_distance')
        self.wp_post_gate_dist  = rospy.get_param(rospy.get_namespace() + 'rrt/waypoint_post_gate_distance')
        self.wp_gate_vel        = rospy.get_param(rospy.get_namespace() + 'rrt/waypoint_gate_velocity')
        self.yawrate_max        = rospy.get_param(rospy.get_namespace() + 'crazyflie/yawrate_max')

        # Publishers
        self.traj_pub = rospy.Publisher(trajectory_topic, Trajectory, queue_size=1)

        # Set up tf stuff
        if tfprefix:
            self.base_frame = tfprefix + '/base_link'
        else:
            self.base_frame = 'base_link'
        self.tf_buff = tf2_ros.Buffer()
        self.tf_lstn = tf2_ros.TransformListener(self.tf_buff)

        # Action server
        self.plan_server = actionlib.SimpleActionServer(
                plan_action, 
                SimpleAction,
                execute_cb=self.plan_cb,
                auto_start=False
        )

        self.map = None
        self.plan_server.start()

        # Parse world map
        rospy.loginfo(rospy.get_name() + ': Creating map...')
        with open(map_file, 'r') as file:
            data = json.load(file)
        self.map = create_map(data, type='grid')
        rospy.loginfo(rospy.get_name() + ': Map created.')

        return

    def plan_cb(self, goal):
        
        # Wait for map to be created
        while not rospy.is_shutdown():
            if self.map is not None:
                break
            rospy.loginfo_throttle(5, 'Planning: Waiting for map...')
            self.wait_rate.sleep()
        rospy.loginfo('Planning: map recieved!')

        # Create waypoints for the planner to pass through
        waypoints = self.create_waypoints(self.map.gates)
        start_wp = waypoints.pop(0)

        # Run path planning
        planner = RRT(self.map)
        traj = planner.plan_traj(start_wp, waypoints)
        while traj is None and not rospy.is_shutdown():
            rospy.logwarn('RRT: Restarting RRT...')
            traj = planner.plan_traj(start_wp, waypoints)

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

        # Set yaw values of traj_msg inplace
        self.set_yaw_gates(traj_msg, start_wp, traj)
        # self.set_yaw_rotating(traj_msg, start_wp, waypoints[-1])
                
        # Publish trajectory
        self.traj_pub.publish(traj_msg)

        # Visualise trajectory in rviz
        if self.use_rviz:
            publish_traj_to_rviz(traj_msg)

        self.plan_server.set_succeeded(SimpleResult(message=''))

    def create_waypoints(self, gates):
        # Get start pose from tf tree
        start_pose = newPoseStamped(0, 0, 0, 0, 0, 0, self.base_frame)
        start_pose = self.tf_buff.transform(start_pose, 'map', rospy.Duration(0.2))

        roll, pitch, yaw = euler_from_quaternion((
                start_pose.pose.orientation.x,
                start_pose.pose.orientation.y,
                start_pose.pose.orientation.z,
                start_pose.pose.orientation.w
        ))
        # Start creating waypoints
        start_wp = Waypoint(
                'start',
                start_pose.pose.position.x,
                start_pose.pose.position.y,
                start_pose.pose.position.z,
                0.0, 0.0, 0.0, 
                yaw
        )

        # Waypoints in front of and after each gate
        waypoints = [start_wp]
        for gate in self.map.gates:
            before_wp, after_wp = self.gate_to_waypoints(gate)
            waypoints.extend([before_wp, after_wp])

        # Last waypoint 0.5 m after last gate
        yaw = self.map.gates[-1]['heading'] * math.pi/180
        normal = Vec3(math.cos(yaw), math.sin(yaw), 0.0)
        pos = self.map.gates[-1]['position'] + 0.5*normal
        vel = Vec3(0.0, 0.0, 0.0)
        last_wp = Waypoint('final', pos.x, pos.y, pos.z, vel.x, vel.y, vel.z, 0.0)
        waypoints.append(last_wp)

        return waypoints

    def gate_to_waypoints(self, gate):
        yaw = gate['heading'] * math.pi/180
        normal = Vec3(math.cos(yaw), math.sin(yaw), 0.0)

        pre_pos  = gate['position'] - self.wp_pre_gate_dist*normal
        post_pos = gate['position'] + self.wp_post_gate_dist*normal

        pre_vel  = self.wp_gate_vel * normal
        post_vel = self.wp_gate_vel * normal

        pre_id   = 'pre_gate_{}'.format(gate['id'])
        post_id  = 'post_gate_{}'.format(gate['id'])

        pre_wp = Waypoint(pre_id, pre_pos.x, pre_pos.y, pre_pos.z, pre_vel.x, pre_vel.y, pre_vel.z, 0.0)
        post_wp = Waypoint(post_id, post_pos.x, post_pos.y, post_pos.z, post_vel.x, post_vel.y, post_vel.z, 0.0)

        return pre_wp, post_wp

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


    def set_yaw_gates(self, traj_msg, start_wp, traj):
        # Add yaw trajectory based on gates
        prev_yaw = (start_wp.yaw *180/math.pi) % 360
        i = 0
        # for gate in self.map.gates[]:
        for gate in self.map.gates[:-1]:
            wp1, wp2 = self.gate_to_waypoints(gate)

            # Loop until pre-gate waypoint
            pre_gate_pieces = []
            duration = 0
            while True:
                bezier = traj[i]
                piece = traj_msg.pieces[i]
                pre_gate_pieces.append(piece)
                duration += piece.duration.to_sec()
                i += 1
                
                wp_pos = Vec3(wp1.x, wp1.y, wp1.z)
                bz_pos = bezier.pos(-1)
                if wp_pos == bz_pos:
                    break
            
            # Calculate yawrate
            gate_yaw = gate['heading']
            if abs(gate_yaw - prev_yaw) <= 180:
                yawrate = (gate_yaw - prev_yaw) / duration
            else:
                yawrate = (360 + gate_yaw - prev_yaw) / duration

            # Turn yaw and yawrate into coefficients
            t = 0
            for piece in pre_gate_pieces:
                piece.poly_yaw[0] = prev_yaw + t*yawrate
                piece.poly_yaw[1] = yawrate
                t += piece.duration.to_sec()

            # Loop until post-gate waypoint
            t = 0
            while True:
                bezier = traj[i]
                piece = traj_msg.pieces[i]
                piece.poly_yaw[0] = gate_yaw
                t += piece.duration.to_sec()
                i += 1
                
                wp_pos = Vec3(wp2.x, wp2.y, wp2.z)
                bz_pos = bezier.pos(-1)
                if wp_pos == bz_pos:
                    break

            prev_yaw = gate_yaw

        # Set yaw values for after the last gate using escaped loop variables
        for piece in traj_msg.pieces[i-1:]:
            piece.poly_yaw[0] = gate_yaw
        
    def set_yaw_rotating(self, traj_msg, start_wp, final_wp):
        prev_yaw = start_wp.yaw * 180/math.pi
        yawrate = 45.0
        for piece in traj_msg.pieces:
            piece.poly_yaw[0] = prev_yaw
            piece.poly_yaw[1] = yawrate
            duration = piece.duration.to_sec()
            prev_yaw = (prev_yaw + duration*yawrate) % 360


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

    dt = 0.1
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
    rospy.spin()




