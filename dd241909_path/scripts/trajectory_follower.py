#!/usr/bin/env python

import math
import numpy as np

import rospy
import tf2_ros
import tf2_geometry_msgs
import actionlib
from nav_msgs.msg import Path
from geometry_msgs.msg import PoseStamped, Pose, Twist, Point, Vector3
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
        tfprefix            = rospy.get_param(rospy.get_name() + '/tfprefix')
        path_topic          = rospy.get_param(rospy.get_name() + '/path_topic')
        trajectory_topic    = rospy.get_param(rospy.get_name() + '/trajectory_topic')
        cmdpos_topic        = rospy.get_param(rospy.get_name() + '/cmdpos_topic')
        cmdfull_topic       = rospy.get_param(rospy.get_name() + '/cmdfull_topic')
        stop_srv_name       = rospy.get_param(rospy.get_name() + '/stop_srv_name')
        navigate_action     = rospy.get_param(rospy.get_name() + '/navigate_action')
        takeoff_action      = rospy.get_param('~takeoff_action')
        rotate_action       = rospy.get_param('~rotate_action')
        hover_action        = rospy.get_param(rospy.get_name() + '/hover_action')
        land_action         = rospy.get_param(rospy.get_name() + '/land_action')

        # Subscribers
        self.path = None
        self.path_sub = rospy.Subscriber(path_topic, Path, self.path_cb)
        self.traj = None
        self.traj_sub = rospy.Subscriber(trajectory_topic, Trajectory, self.traj_cb)
        # self.has_seen_marker = False
        # self.seen_marker_sub = rospy.Subscriber('aruco/result', Trajectory, self.traj_cb)

        # Publishers
        self.cmdpos_pub = rospy.Publisher(cmdpos_topic, Position, queue_size=1)
        self.cmdfull_pub = rospy.Publisher(cmdfull_topic, FullState, queue_size=1)

        # Set up tf stuff
        self.base_frame = 'base_link'
        self.odom_frame = 'odom'
        if tfprefix:
            self.base_frame = tfprefix + '/' + self.base_frame
            self.odom_frame = tfprefix + '/' + self.odom_frame
        self.tf_buff = tf2_ros.Buffer()
        self.tf_lstn = tf2_ros.TransformListener(self.tf_buff)

        # Service clients
        rospy.wait_for_service(stop_srv_name)
        self.stop_srv = rospy.ServiceProxy(stop_srv_name, Stop)

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
            while not self.path:
                rospy.loginfo_throttle(10, rospy.get_name() + ': Waiting for path...')
                self.wait_rate.sleep()
            rospy.loginfo(rospy.get_name() + ': Path recieved.')

            rospy.loginfo(rospy.get_name() + ': Off we go!')
            path = self.path
            for target in path.poses:
                rospy.loginfo(rospy.get_name() + ': Moving to new pose...')
                current_pose = self.get_base_pose()
                while pose_dist(target.pose, current_pose.pose) >= self.tol:
                    target.header.stamp = rospy.Time.now()
                    self.cmdpos_pub.publish(target)
                    current_pose = self.get_base_pose()
                    # TODO: Check preemption
                    self.cmd_rate.sleep()

            rospy.loginfo(rospy.get_name() + ': Path Completed!')
            self.nav_server.set_succeeded(NavigateResult(message=''))
        except ROSInterruptException:
            self.nav_server.set_aborted(NavigateResult(message=''))

    def navigate_traj_cb(self, goal):
        try:
            while not self.traj:
                rospy.loginfo_throttle(10, rospy.get_name() + ': Waiting for trajectory...')
                self.wait_rate.sleep()
            rospy.loginfo(rospy.get_name() + ': Trajectory recieved.')

            # Start of trajectory
            t0 = rospy.Time.now()
            # Accumulated duration of passed trajectory pieces
            passed_t = rospy.Duration(0)

            rospy.loginfo(rospy.get_name() + ': Off we go!')
            for piece in self.traj.pieces:
                rospy.loginfo(rospy.get_name() + ': New trajectory piece!')

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
                while t <= piece.duration:
                    t_now = rospy.Time.now()
                    t = t_now - t0 - passed_t
                    tsec = t.to_sec()
                    tv = np.power([tsec]*nrterms, range(nrterms))
                    posyaw = np.dot(C_pos, tv)
                    velyaw = np.dot(C_vel, tv)
                    accyaw = np.dot(C_acc, tv)

                    msg = FullState()
                    msg.header.frame_id = self.traj.header.frame_id
                    msg.header.stamp = t_now
                    yaw = posyaw[3] % 360
                    msg.pose  = newPose(posyaw[0], posyaw[1], posyaw[2], yaw, yaw, yaw)
                    msg.twist = newTwist(velyaw[0], velyaw[1], velyaw[2], velyaw[3], velyaw[3], velyaw[3])
                    msg.acc   = Vector3(accyaw[0], accyaw[1], accyaw[2])

                    self.cmdfull_pub.publish(msg)
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

            while not rospy.is_shutdown():
                msg.header.stamp = rospy.Time.now()
                self.cmdpos_pub.publish(msg)

                # TODO: Check preemption

                dist = pose_dist(target_pose.pose, self.get_base_pose(frame_id=self.odom_frame).pose)
                if dist < 0.05:
                    self.takeoff_server.set_succeeded(SimpleResult(message=''))
                    return
                else:
                    rospy.loginfo_throttle(1, 'Takeoff: dist = {}'.format(round(dist, 4)))
                self.cmd_rate.sleep()
                
                
        except ROSInterruptException:
            self.takeoff_server.set_aborted(SimpleResult(message=''))

    def rotate_cb(self, goal):
        try:
            rospy.loginfo(rospy.get_name() + ': Start rotate!')
            target_pose = self.get_base_pose(frame_id=self.odom_frame)
            msg = cfposition_from_pose(target_pose)
            msg.z = 0.4

            while not rospy.is_shutdown():
                rospy.loginfo_throttle(5, 'Rotating...')
                
                msg.header.stamp = rospy.Time.now()
                self.cmdpos_pub.publish(msg)

                msg.yaw += 0.5
                if msg.yaw >= 360.0:
                    msg.yaw -= 360.0

                if self.rotate_server.is_preempt_requested():
                    rospy.loginfo(rospy.get_name() + ': Rotate preempted!')
                    self.rotate_server.set_preempted(SimpleResult(message=''))
                    return

                self.cmd_rate.sleep()
                
        except ROSInterruptException:
            self.rotate_server.set_aborted(SimpleResult(message=''))

    def hover_cb(self, goal):
        try:
            rospy.loginfo(rospy.get_name() + ': Hovering...')
            target_pose = self.get_base_pose()
            msg = cfposition_from_pose(target_pose)
            
            msg.z = max(msg.z, 0.4)

            rospy.loginfo(rospy.get_name() + ': ... at ({}, {}, {}, {})'.format(msg.x, msg.y, msg.z, msg.yaw))

            while not rospy.is_shutdown():
                rospy.loginfo_throttle(10, rospy.get_name() + ': Hovering...')

                msg.header.stamp = rospy.Time.now()
                self.cmdpos_pub.publish(msg)

                if self.hover_server.is_preempt_requested():
                    rospy.loginfo(rospy.get_name() + ': Hover preempted!')
                    self.hover_server.set_preempted(SimpleResult(message=''))
                    return
                
                self.cmd_rate.sleep()
                
        except ROSInterruptException:
            self.hover_server.set_aborted(SimpleResult(message=''))

    def land_cb(self, goal):
        try:
            rospy.loginfo(rospy.get_name() + ': Landing...')
            current_pose = self.get_base_pose()
            msg = cfposition_from_pose(current_pose)

            while not rospy.is_shutdown() and current_pose.pose.position.z > 0.1:
                current_pose = self.get_base_pose()
                z_error = current_pose.pose.position.z - 0.1
                if abs(z_error) < 0.05:
                    msg.z = 0.1
                else:
                    msg.z = current_pose.pose.position.z - math.copysign(0.05, z_error)

                msg.header.stamp = rospy.Time.now()
                self.cmdpos_pub.publish(msg)
                
                if self.land_server.is_preempt_requested():
                    rospy.loginfo(rospy.get_name() + ': Land preempted!')
                    self.land_server.set_preempted(SimpleResult(message=''))
                    return

                self.cmd_rate.sleep()

            rospy.loginfo(rospy.get_name() + ': Calling stop service...')
            self.stop_srv(0)
            rospy.loginfo(rospy.get_name() + ': Landing completed.')

            self.land_server.set_succeeded(SimpleResult(message=''))
        except ROSInterruptException:
            self.land_server.set_aborted(NavigateResult(message=''))

    def get_base_pose(self, frame_id='map'):
        current_pose = newPoseStamped(0, 0, 0, 0, 0, 0, self.base_frame)
        while not rospy.is_shutdown():
            try:
                return self.tf_buff.transform(current_pose, frame_id, rospy.Duration(0.05))
            except ExtrapolationException:
                rospy.logwarn_throttle(1.0, rospy.get_name() + ': Cannot transform from {} to {}'.format(self.base_frame, frame_id))
                self.wait_rate.sleep()



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

def cfposition_from_pose(pose):
    # Convert to crazyflie position msg (x,y,z,yaw)
    roll, pitch, yaw = euler_from_quaternion((
            pose.pose.orientation.x,
            pose.pose.orientation.y,
            pose.pose.orientation.z,
            pose.pose.orientation.w
    ))
    # NOTE Yaw is in degrees.
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

