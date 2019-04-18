#!/usr/bin/env python

from __future__ import division

import math
import numpy as np

import rospy
import tf2_ros
import tf2_geometry_msgs
import actionlib
from std_msgs.msg import Empty
from nav_msgs.msg import Path
from geometry_msgs.msg import PoseStamped, Pose, Twist, Point, Vector3, Quaternion
from tf.transformations import quaternion_from_euler, euler_from_quaternion
from rospy.exceptions import ROSInterruptException
from tf2_ros import ExtrapolationException

from crazyflie_driver.msg import FullState, Position
from crazyflie_driver.srv import Stop

from dd241909_msgs.msg import Trajectory
from dd241909_msgs.msg import NavigateAction, NavigateResult
from dd241909_msgs.msg import SimpleAction, SimpleResult


class TrajectoryFollower(object):

    tol = 0.08

    def __init__(self):
        rospy.init_node('trajectory_follower', log_level=rospy.INFO)
        rospy.loginfo(rospy.get_name() + ': Initialising node...')

        self.wait_rate = rospy.Rate(10) # Wait for stuff at this rate
        self.cmd_rate = rospy.Rate(20) # Send commands at this rate

        # Access ros parameters
        tfprefix            = rospy.get_param('~tfprefix')
        path_topic          = rospy.get_param('~path_topic')
        trajectory_topic    = rospy.get_param('~trajectory_topic')
        cmdstop_topic       = rospy.get_param('~cmdstop_topic')
        cmdpos_topic        = rospy.get_param('~cmdpos_topic')
        cmdfull_topic       = rospy.get_param('~cmdfull_topic')
        stop_srv_name       = rospy.get_param('~stop_srv_name')
        navigate_action     = rospy.get_param('~navigate_action')
        takeoff_action      = rospy.get_param('~takeoff_action')
        rotate_action       = rospy.get_param('~rotate_action')
        hover_action        = rospy.get_param('~hover_action')
        land_action         = rospy.get_param('~land_action')

        # Subscribers
        self.path = None
        self.path_sub = rospy.Subscriber(path_topic, Path, self.path_cb)
        self.traj = None
        self.traj_sub = rospy.Subscriber(trajectory_topic, Trajectory, self.traj_cb)

        # Publishers
        self.cmdstop_pub = rospy.Publisher(cmdstop_topic, Empty, queue_size=1)
        self.cmdpos_pub = rospy.Publisher(cmdpos_topic, Position, queue_size=1)
        self.cmdfull_pub = rospy.Publisher(cmdfull_topic, FullState, queue_size=1)

        # Send stop command on shutdown
        def stop():
            rospy.loginfo('Shutdown detected. Sending stop command!')
            self.cmdstop_pub.publish(Empty())
        rospy.on_shutdown(stop)

        # Set up tf stuff
        self.base_frame = 'base_link'
        self.odom_frame = 'odom'
        if tfprefix:
            self.base_frame = tfprefix + '/' + self.base_frame
            self.odom_frame = tfprefix + '/' + self.odom_frame
        self.tf_buff = tf2_ros.Buffer()
        self.tf_lstn = tf2_ros.TransformListener(self.tf_buff)

        # Service clients
        # rospy.wait_for_service(stop_srv_name)
        # self.stop_srv = rospy.ServiceProxy(stop_srv_name, Stop)

        # Action servers
        self.nav_server = actionlib.SimpleActionServer(
                navigate_action, 
                NavigateAction, 
                #execute_cb=self.navigate_path_cb,
                execute_cb=self.navigate_traj_cb,
                auto_start=False
        )
        self.takeoff_server = actionlib.SimpleActionServer(
                takeoff_action, 
                SimpleAction, 
                execute_cb=self.takeoff_cb,
                auto_start=False
        )
        self.rotate_server = actionlib.SimpleActionServer(
                rotate_action, 
                SimpleAction, 
                execute_cb=self.rotate_cb,
                auto_start=False
        )
        self.hover_server = actionlib.SimpleActionServer(
                hover_action, 
                SimpleAction, 
                execute_cb=self.hover_cb,
                auto_start=False
        )
        self.land_server = actionlib.SimpleActionServer(
                land_action, 
                SimpleAction, 
                execute_cb=self.land_cb,
                auto_start=False
        )

        # Start action servers
        rospy.loginfo(rospy.get_name() + ': Starting {} action server...'.format(navigate_action))
        self.nav_server.start()
        rospy.loginfo(rospy.get_name() + ': Starting {} action server...'.format(takeoff_action))
        self.takeoff_server.start()
        rospy.loginfo(rospy.get_name() + ': Starting {} action server...'.format(rotate_action))
        self.rotate_server.start()
        rospy.loginfo(rospy.get_name() + ': Starting {} action server...'.format(hover_action))
        self.hover_server.start()
        rospy.loginfo(rospy.get_name() + ': Starting {} action server...'.format(land_action))
        self.land_server.start()

    def path_cb(self, msg):
        self.path = msg

    def traj_cb(self, msg):
        self.traj = msg

    def navigate_path_cb(self, goal):
        try:
            # Wait for a path to be published
            while not self.path:
                if self.nav_server.is_preempt_requested():
                    rospy.loginfo(rospy.get_name() + ': Navigation preempted!')
                    self.nav_server.set_preempted(SimpleResult(message=''))
                    return
                rospy.loginfo_throttle(10, rospy.get_name() + ': Waiting for path...')
                self.wait_rate.sleep()
            rospy.loginfo(rospy.get_name() + ': Path recieved.')

            rospy.loginfo(rospy.get_name() + ': Off we go!')
            for target_pose in self.path.poses:
                rospy.loginfo(rospy.get_name() + ': Moving to new pose...')
                while not rospy.is_shutdown():
                    if self.nav_server.is_preempt_requested():
                        rospy.loginfo(rospy.get_name() + ': Navigation preempted!')
                        self.nav_server.set_preempted(SimpleResult(message=''))
                        return

                    dist = pose_dist(target_pose.pose, self.get_base_pose().pose)
                    if dist < self.tol:
                        break

                    target_pose.header.stamp = rospy.Time.now()
                    self.cmdpos_pub.publish(target_pose)
                    self.cmd_rate.sleep()

            rospy.loginfo(rospy.get_name() + ': Path Completed!')
            self.nav_server.set_succeeded(NavigateResult(message=''))
        except ROSInterruptException:
            self.nav_server.set_aborted(NavigateResult(message=''))

    def navigate_traj_cb(self, goal):
        try:
            # Wait for a trajectory to be published
            while not self.traj:
                if self.nav_server.is_preempt_requested():
                    rospy.loginfo(rospy.get_name() + ': Navigation preempted!')
                    self.nav_server.set_preempted(SimpleResult(message=''))
                    return
                rospy.loginfo_throttle(10, rospy.get_name() + ': Waiting for trajectory...')
                self.wait_rate.sleep()
            rospy.loginfo(rospy.get_name() + ': Trajectory recieved.')

            # Start of trajectory
            t0 = rospy.Time.now()
            # Accumulated duration of passed trajectory pieces
            passed_t = rospy.Duration(0)

            rospy.loginfo(rospy.get_name() + ': Off we go!')
            for piece in self.traj.pieces:
                if self.nav_server.is_preempt_requested():
                    rospy.loginfo(rospy.get_name() + ': Navigation preempted!')
                    self.nav_server.set_preempted(SimpleResult(message=''))
                    return
                rospy.logdebug(rospy.get_name() + ': New trajectory piece!')

                nrterms = len(piece.poly_x)

                # Position coefficients
                C_pos = np.zeros((4, nrterms))
                for i in range(nrterms):
                    C_pos[0, i] = piece.poly_x[i]
                    C_pos[1, i] = piece.poly_y[i]
                    C_pos[2, i] = piece.poly_z[i]
                    C_pos[3, i] = piece.poly_yaw[i]

                # Velocity coefficients
                C_vel = np.zeros((4, nrterms))
                for i in range(nrterms-1):
                    C_vel[0, i] = (i+1) * C_pos[0, i+1]
                    C_vel[1, i] = (i+1) * C_pos[1, i+1]
                    C_vel[2, i] = (i+1) * C_pos[2, i+1]
                    C_vel[3, i] = (i+1) * C_pos[3, i+1]

                # Acceleration coefficients
                C_acc = np.zeros((4, nrterms))
                for i in range(nrterms-2):
                    C_acc[0, i] = (i+1) * C_vel[0,i+1]
                    C_acc[1, i] = (i+1) * C_vel[1,i+1]
                    C_acc[2, i] = (i+1) * C_vel[2,i+1]
                    C_acc[3, i] = (i+1) * C_vel[3,i+1]

                t = rospy.Time.now() - t0 - passed_t # For first comparision
                while not rospy.is_shutdown() and t <= piece.duration:
                    if self.nav_server.is_preempt_requested():
                        rospy.loginfo(rospy.get_name() + ': Navigation preempted!')
                        self.nav_server.set_preempted(SimpleResult(message=''))
                        return
                    
                    t_now = rospy.Time.now()
                    t = t_now - t0 - passed_t
                    tsec = t.to_sec()
                    tv = np.power([tsec]*nrterms, range(nrterms))
                    posyaw = np.dot(C_pos, tv)
                    velyaw = np.dot(C_vel, tv)
                    accyaw = np.dot(C_acc, tv)

                    # msg = FullState()
                    # msg.header.frame_id = self.traj.header.frame_id
                    # msg.header.stamp = t_now
                    # yaw = posyaw[3] % 360
                    # msg.pose  = newPose(posyaw[0], posyaw[1], posyaw[2], 0.0, 0.0, yaw)
                    # msg.twist = newTwist(velyaw[0], velyaw[1], velyaw[2], 0.0, 0.0, velyaw[3])
                    # msg.acc   = Vector3(accyaw[0], accyaw[1], accyaw[2])

                    # self.cmdfull_pub.publish(msg)
                    # self.cmd_rate.sleep()

                    target_pose = newPoseStamped(posyaw[0], posyaw[1], posyaw[2], 0.0, 0.0, posyaw[3] * math.pi/180.0, 'map', stamp=rospy.Time())
                    target_pose = self.tf_buff.transform(target_pose, self.odom_frame)
                    msg = cfposition_from_pose(target_pose)

                    self.cmdpos_pub.publish(msg)
                    self.cmd_rate.sleep()

                # When piece is done update passed_t
                passed_t += piece.duration

            rospy.loginfo(rospy.get_name() + ': Trajectory completed!')
            self.nav_server.set_succeeded(NavigateResult(message=''))
        except ROSInterruptException:
            self.nav_server.set_aborted(NavigateResult(message=''))

    def takeoff_cb(self, goal):
        try:
            rospy.loginfo(rospy.get_name() + ': Takeoff!')
            target_pose = self.get_base_pose(frame_id=self.odom_frame)
            msg = cfposition_from_pose(target_pose)
            msg.z = 0.4

            # Move close to target pose
            while not rospy.is_shutdown():
                if self.takeoff_server.is_preempt_requested():
                    rospy.loginfo(rospy.get_name() + ': Takeoff preempted!')
                    self.takeoff_server.set_preempted(SimpleResult(message=''))
                    return

                dist = pose_dist(target_pose.pose, self.get_base_pose(frame_id=self.odom_frame).pose)
                if dist < self.tol:
                    break

                msg.header.stamp = rospy.Time.now()
                self.cmdpos_pub.publish(msg)
                self.cmd_rate.sleep()
            
            # Wait a bit to stabilise before exiting
            for _ in range(40):
                if self.takeoff_server.is_preempt_requested():
                    rospy.loginfo(rospy.get_name() + ': Takeoff preempted!')
                    self.takeoff_server.set_preempted(SimpleResult(message=''))
                    return

                msg.header.stamp = rospy.Time.now()
                self.cmdpos_pub.publish(msg)
                self.cmd_rate.sleep()

            rospy.loginfo(rospy.get_name() + ': Takeoff completed!')
            self.takeoff_server.set_succeeded(SimpleResult(message=''))
        except ROSInterruptException:
            self.takeoff_server.set_aborted(SimpleResult(message=''))

    def rotate_cb(self, goal):
        try:
            rospy.loginfo(rospy.get_name() + ': Start rotate!')
            target_pose = self.get_base_pose(frame_id=self.odom_frame)
            target_pose.pose.position.z = 0.4
            msg = cfposition_from_pose(target_pose)

            while not rospy.is_shutdown():
                if self.rotate_server.is_preempt_requested():
                    rospy.loginfo(rospy.get_name() + ': Rotate preempted!')
                    self.rotate_server.set_preempted(SimpleResult(message=''))
                    return

                rospy.loginfo_throttle(5, rospy.get_name() + ': Rotating...')

                msg.yaw = (msg.yaw + 0.5) % 360.0
                msg.header.stamp = rospy.Time.now()
                self.cmdpos_pub.publish(msg)
                self.cmd_rate.sleep()
            
            rospy.loginfo(rospy.get_name() + ': Rotate detected shutdown. Aborting...')
            self.rotate_server.set_aborted(SimpleResult(message=''))
        except ROSInterruptException:
            self.rotate_server.set_aborted(SimpleResult(message=''))

    def hover_cb(self, goal):
        try:
            rospy.loginfo(rospy.get_name() + ': Hovering...')
            target_pose = self.get_base_pose(frame_id=self.odom_frame)
            msg = cfposition_from_pose(target_pose)
            msg.z = max(msg.z, 0.4)

            rospy.loginfo(rospy.get_name() + ': ... at ({}, {}, {}, {})'.format(msg.x, msg.y, msg.z, msg.yaw))

            while not rospy.is_shutdown():
                if self.hover_server.is_preempt_requested():
                    rospy.loginfo(rospy.get_name() + ': Hover preempted!')
                    self.hover_server.set_preempted(SimpleResult(message=''))
                    return

                rospy.loginfo_throttle(10, rospy.get_name() + ': Hovering...')

                msg.header.stamp = rospy.Time.now()
                self.cmdpos_pub.publish(msg)
                self.cmd_rate.sleep()
            
            rospy.loginfo(rospy.get_name() + ': Hover detected shutdown. Aborting...')
            self.hover_server.set_aborted(SimpleResult(message='')) 
        except ROSInterruptException:
            self.hover_server.set_aborted(SimpleResult(message=''))

    def land_cb(self, goal):
        try:
            rospy.loginfo(rospy.get_name() + ': Landing...')
            current_pose = self.get_base_pose(frame_id=self.odom_frame)
            msg = cfposition_from_pose(current_pose)

            while not rospy.is_shutdown() and current_pose.pose.position.z > 0.1:
                if self.land_server.is_preempt_requested():
                    rospy.loginfo(rospy.get_name() + ': Land preempted!')
                    self.land_server.set_preempted(SimpleResult(message=''))
                    return

                current_pose = self.get_base_pose(frame_id=self.odom_frame)
                z_error = current_pose.pose.position.z - 0.1
                if abs(z_error) < 0.05:
                    msg.z = 0.1
                else:
                    msg.z = current_pose.pose.position.z - math.copysign(0.05, z_error)

                msg.header.stamp = rospy.Time.now()
                self.cmdpos_pub.publish(msg)
                self.cmd_rate.sleep()

            rospy.loginfo(rospy.get_name() + ': Calling stop service...')
            #self.stop_srv(0)
            self.cmdstop_pub.publish(Empty())
            rospy.loginfo(rospy.get_name() + ': Landing completed.')

            self.land_server.set_succeeded(SimpleResult(message=''))
        except ROSInterruptException:
            self.land_server.set_aborted(NavigateResult(message=''))

    def get_base_pose(self, frame_id='map'):
        current_pose = newPoseStamped(0, 0, 0, 0, 0, 0, self.base_frame, stamp=rospy.Time())
        # while not rospy.is_shutdown():
        #     try:
        #         return self.tf_buff.transform(current_pose, frame_id, rospy.Duration(0.05))
        #     except ExtrapolationException:
        #         rospy.logwarn_throttle(1.0, rospy.get_name() + ': Cannot transform from {} to {}'.format(self.base_frame, frame_id))
        #         self.wait_rate.sleep()

        return self.tf_buff.transform_full(current_pose, frame_id, rospy.Time(), fixed_frame='map')


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

def newPose(x, y, z, roll, pitch, yaw):
    pose = Pose()
    pose.position = Point(x=x, y=y, z=z)
    (pose.orientation.x,
     pose.orientation.y,
     pose.orientation.z,
     pose.orientation.w) = quaternion_from_euler(roll, pitch, yaw)
    return pose

def newTwist(x, y, z, roll, pitch, yaw):
    twist = Twist() 
    twist.linear.x = x
    twist.linear.y = y
    twist.linear.z = z
    twist.angular.x = roll
    twist.angular.y = pitch
    twist.angular.z = yaw
    return twist

def newQuaternion(roll, pitch, yaw, radians=True):
    if not radians:
        roll = roll * math.pi/180.0
        pitch = pitch * math.pi/180.0
        yaw = yaw * math.pi/180.0

    q = Quaternion()
    q.x, q.y, q.z, q.w = quaternion_from_euler(roll, pitch, yaw)
    return q

def cfposition_from_pose(pose):
    roll, pitch, yaw = euler_from_quaternion((
            pose.pose.orientation.x,
            pose.pose.orientation.y,
            pose.pose.orientation.z,
            pose.pose.orientation.w
    ))
    position = Position(
            pose.header,
            pose.pose.position.x,
            pose.pose.position.y,
            pose.pose.position.z,
            yaw*180.0/math.pi
    )
    return position

def point_dist(p1, p2):
    return math.sqrt((p1.x-p2.x)**2 + (p1.y-p2.y)**2 + (p1.z-p2.z)**2)

def quat_dist(q1, q2):
    return 1 - (q1.x*q2.x + q1.y*q2.y + q1.z*q2.z + q1.w*q2.w)**2

def pose_dist(pose1, pose2):
    if type(pose1) == Pose:
        p1 = pose1.position
        q1 = pose1.orientation
    else:
        p1 = pose1.translation
        q1 = pose1.rotation
    if type(pose2) == Pose:
        p2 = pose2.position
        q2 = pose2.orientation
    else:
        p2 = pose2.translation
        q2 = pose2.rotation
    return point_dist(p1, p2) + quat_dist(q1, q2)


if __name__ == '__main__':
    try:
        follower = TrajectoryFollower()
    except ROSInterruptException:
        pass
    
    rospy.spin()

