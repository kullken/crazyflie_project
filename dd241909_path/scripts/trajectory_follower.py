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


class TrajectoryFollower(object):

    tol = 0.07

    def __init__(self):
        rospy.init_node('trajectory_follower', log_level=rospy.INFO)
        rospy.loginfo(rospy.get_name() + ': Initialising node...')

        self.rate = rospy.Rate(10)

        # Access ros parameters
        tfprefix         = rospy.get_param(rospy.get_name() + '/tfprefix')
        goal_topic       = rospy.get_param(rospy.get_name() + '/navgoal_topic')
        trajectory_topic = rospy.get_param(rospy.get_name() + '/trajectory_topic')
        navigate_action  = rospy.get_param(rospy.get_name() + '/navigate_action')

        # Subscribers
        self.traj_sub = rospy.Subscriber(trajectory_topic, Path, self.traj_cb)

        # Publishers
        self.goal_pub = rospy.Publisher(goal_topic, PoseStamped, queue_size=2)

        # Set up tf stuff
        if tfprefix:
            self.base_frame = tfprefix + '/base_link'
        else:
            self.base_frame = 'base_link'
        self.tf_buff = tf2_ros.Buffer()
        self.tf_lstn = tf2_ros.TransformListener(self.tf_buff)

        # Action server
        self.nav_server = actionlib.SimpleActionServer(navigate_action, 
                                                       NavigateAction, 
                                                       execute_cb=self.navigate_cb,
                                                       auto_start=False
                                                       )

        # Wait for first message
        rospy.loginfo(rospy.get_name() + ': Waiting for trajectory...')
        rospy.wait_for_message(trajectory_topic, Path)
        rospy.loginfo(rospy.get_name() + ': Trajectory recieved.')

        
        # Initialisation done, start action server
        rospy.loginfo(rospy.get_name() + ': Starting {} action server...'.format(navigate_action))
        self.nav_server.start()

    def traj_cb(self, msg):
        rospy.loginfo(rospy.get_name() + ': New trajectory recieved.')
        self.traj = msg

    def navigate_cb(self, goal):
        if not goal.start:
            # goal.start == False is conceptually unneeded, perhaps replace with Empty msg-type?
            return

        rospy.loginfo(rospy.get_name() + ': Off we go!')
        for target in self.traj.poses:
            rospy.loginfo(rospy.get_name() + ': Moving to new pose...')

            current_pose = self.get_base_pose()

            rospy.logwarn(rospy.get_name() + 'Distance to target = {}'.format(pose_dist(target.pose, current_pose.pose)))
            while pose_dist(target.pose, current_pose.pose) >= self.tol:
                rospy.logwarn(rospy.get_name() 
                        + ': Target: \n{} \nCurrent: \n{}'.format(target.pose.position, current_pose.pose.position))
                rospy.logwarn(rospy.get_name() 
                        + ': Point distance = {}'.format(point_dist(target.pose.position, current_pose.pose.position)))
                rospy.logwarn(rospy.get_name() 
                        + ': Quaternion distance = {}'.format(quat_dist(target.pose.orientation, current_pose.pose.orientation)))
                rospy.logwarn(rospy.get_name() 
                        + ': Distance to target = {}'.format(pose_dist(target.pose, current_pose.pose)))

                target.header.stamp = rospy.Time.now()
                self.goal_pub.publish(target)

                current_pose = self.get_base_pose()

                # TODO: Check preemption
                self.rate.sleep()

        rospy.loginfo(rospy.get_name() + ': Trajectory done!')
        self.nav_server.set_succeeded(NavigateResult(success=True))

    def get_base_pose(self):
        current_pose = newPoseStamped(0, 0, 0, 0, 0, 0, self.base_frame)
        rate = rospy.Rate(20)   # 20 Hz, maximum rate of while loop
        while not rospy.is_shutdown():
            try:
                return self.tf_buff.transform(current_pose, 'map', rospy.Duration(0.05))
            except ExtrapolationException:
                rospy.logwarn_throttle(1.0, rospy.get_name() + 'cannot transform from {} to map'.format(self.base_frame))
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

