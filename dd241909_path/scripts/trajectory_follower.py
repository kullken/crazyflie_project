#!/usr/bin/env python

import math

import rospy
import tf2_ros
import tf2_geometry_msgs
import actionlib
from nav_msgs.msg import Path
from geometry_msgs.msg import PoseStamped, Pose, Point
from tf.transformations import quaternion_from_euler
from rospy.exceptions import ROSInterruptException
from tf2_ros import ExtrapolationException

from dd241909_msgs.msg import NavigateAction, NavigateResult
from dd241909_msgs.msg import SimpleAction, SimpleResult


class TrajectoryFollower(object):

    tol = 0.08

    def __init__(self):
        rospy.init_node('trajectory_follower', log_level=rospy.INFO)
        rospy.loginfo(rospy.get_name() + ': Initialising node...')

        self.rate = rospy.Rate(10)

        # Access ros parameters
        tfprefix            = rospy.get_param(rospy.get_name() + '/tfprefix')
        goal_topic          = rospy.get_param(rospy.get_name() + '/navgoal_topic')
        trajectory_topic    = rospy.get_param(rospy.get_name() + '/trajectory_topic')
        navigate_action     = rospy.get_param(rospy.get_name() + '/navigate_action')
        hover_action        = rospy.get_param(rospy.get_name() + '/hover_action')
        land_action         = rospy.get_param(rospy.get_name() + '/land_action')

        # Subscribers
        self.traj_sub = rospy.Subscriber(trajectory_topic, Path, self.traj_cb)

        # Publishers
        self.goal_pub = rospy.Publisher(goal_topic, PoseStamped, queue_size=1)

        # Set up tf stuff
        if tfprefix:
            self.base_frame = tfprefix + '/base_link'
        else:
            self.base_frame = 'base_link'
        self.tf_buff = tf2_ros.Buffer()
        self.tf_lstn = tf2_ros.TransformListener(self.tf_buff)

        # Action servers
        self.nav_server = actionlib.SimpleActionServer(
            navigate_action, 
                                                       NavigateAction, 
                                                       execute_cb=self.navigate_cb,
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

        # Hover and land servers can start immedietly
        rospy.loginfo(rospy.get_name() + ': Starting {} action server...'.format(hover_action))
        self.hover_server.start()
        rospy.loginfo(rospy.get_name() + ': Starting {} action server...'.format(land_action))
        self.land_server.start()

        # Wait for first message before starting navigation server
        rospy.loginfo(rospy.get_name() + ': Waiting for trajectory...')
        rospy.wait_for_message(trajectory_topic, Path)
        rospy.loginfo(rospy.get_name() + ': Trajectory recieved.')

        rospy.loginfo(rospy.get_name() + ': Starting {} action server...'.format(navigate_action))
        self.nav_server.start()

    def traj_cb(self, msg):
        rospy.loginfo(rospy.get_name() + ': New trajectory recieved.')
        self.traj = msg
        # TODO: Reset loop in naviagte_cb since trajectory has been updated

    def navigate_cb(self, goal):
        try:
            if not goal.start:
                # goal.start == False is conceptually unneeded, perhaps replace with Empty msg-type?
                self.nav_server.set_succeeded(NavigateResult(success=True)) # Successfully avoided doing anything
                return

            rospy.loginfo(rospy.get_name() + ': Off we go!')
            for target in self.traj.poses:
                rospy.loginfo(rospy.get_name() + ': Moving to new pose...')
                current_pose = self.get_base_pose()
                while pose_dist(target.pose, current_pose.pose) >= self.tol:
                    target.header.stamp = rospy.Time.now()
                    self.goal_pub.publish(target)
                    current_pose = self.get_base_pose()
                    # TODO: Check preemption
                    self.rate.sleep()

            rospy.loginfo(rospy.get_name() + ': Trajectory done!')
            self.nav_server.set_succeeded(NavigateResult(success=True))
        except ROSInterruptException:
            self.nav_server.set_succeeded(NavigateResult(success=False))

    def hover_cb(self, goal):
        try:
            rospy.loginfo(rospy.get_name() + ': Hovering...')
            target = self.get_base_pose()
            if target.pose.position.z < 0.2:
                target.pose.position.z = 0.2

            while not rospy.is_shutdown():
                    target.header.stamp = rospy.Time.now()
                    self.goal_pub.publish(target)
                # TODO: Check preemption
                self.rate.sleep()
                
        except ROSInterruptException:
            pass

    def land_cb(self, goal):
        try:
            rospy.loginfo(rospy.get_name() + ': Landing...')
                    current_pose = self.get_base_pose()
            target = current_pose

            while not rospy.is_shutdown() and current_pose.pose.position.z > 0.1:
                current_pose = self.get_base_pose()
                target.pose.position.z = current_pose.pose.position.z - 0.05
                target.header.stamp = rospy.Time.now()
                self.goal_pub.publish(target)
                    # TODO: Check preemption
                    self.rate.sleep()

            self.land_server.set_succeeded(SimpleResult(success=True))
        except ROSInterruptException:
            pass

    def get_base_pose(self):
        current_pose = newPoseStamped(0, 0, 0, 0, 0, 0, self.base_frame)
        rate = rospy.Rate(20)   # 20 Hz, maximum rate of while loop
        while not rospy.is_shutdown():
            try:
                return self.tf_buff.transform(current_pose, 'map', rospy.Duration(0.05))
            except ExtrapolationException:
                rospy.logwarn_throttle(1.0, rospy.get_name() + ': Cannot transform from {} to map'.format(self.base_frame))
                rate.sleep()




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

