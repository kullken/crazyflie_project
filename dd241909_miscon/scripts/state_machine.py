#!/usr/bin/env python

import math
from collections import namedtuple
State = namedtuple('State', ['name', 'setup', 'action'])

import rospy
import tf2_ros
import tf2_geometry_msgs
from rospy.exceptions import ROSInterruptException
from tf.transformations import quaternion_from_euler
from geometry_msgs.msg import PoseStamped, Point

def point_dist(p1, p2):
    return math.sqrt((p1.x-p2.x)**2 + (p1.y-p2.y)**2 + (p1.z-p2.z)**2)

class StateMachine(object):

    tol = 0.1

    done_state      = 0
    setup_state     = 1000
    marker0_state   = 1011
    marker1_state   = 1012
    marker2_state   = 1013
    stopsign_state  = 1014

    def __init__(self):

        rospy.init_node('state_machine')
        self.rate = rospy.Rate(10)  # Hz
        self.goal_pub = rospy.Publisher('/cf1/move_base_simple/goal', PoseStamped, queue_size=5)

        self.tf_buff = tf2_ros.Buffer()
        self.tf_list = tf2_ros.TransformListener(self.tf_buff)

        self.state = self.setup_state

        # marker0 observation pose
        pose = PoseStamped()
        pose.header.frame_id = 'map'
        pose.pose.position = Point(x=0, y=0, z=0.5)
        (pose.pose.orientation.x,
         pose.pose.orientation.y,
         pose.pose.orientation.z,
         pose.pose.orientation.w) = quaternion_from_euler(0, 0, 0)
        self.marker0_obs_pose = pose

        # marker1 observation pose
        pose = PoseStamped()
        pose.header.frame_id = 'map'
        pose.pose.position = Point(x=4.3, y=0, z=0.5)
        (pose.pose.orientation.x,
         pose.pose.orientation.y,
         pose.pose.orientation.z,
         pose.pose.orientation.w) = quaternion_from_euler(0, 0, 0)
        self.marker1_obs_pose = pose

        # marker2 observation pose
        pose = PoseStamped()
        pose.header.frame_id = 'map'
        pose.pose.position = Point(6, 0, 0.5)
        (pose.pose.orientation.x,
         pose.pose.orientation.y,
         pose.pose.orientation.z,
         pose.pose.orientation.w) = quaternion_from_euler(0, 0, 0)
        self.marker2_obs_pose = pose

        # stopsign observation pose
        pose = PoseStamped()
        pose.header.frame_id = 'map'
        pose.pose.position = Point(8, 0, 0.5)
        (pose.pose.orientation.x,
         pose.pose.orientation.y,
         pose.pose.orientation.z,
         pose.pose.orientation.w) = quaternion_from_euler(0, 0, 0)
        self.stopsign_obs_pose = pose

    @property
    def state(self):
        return self._state

    @state.setter
    def state(self, new_state):
        rospy.logwarn('State machine changing state to: ' + str(new_state))
        self._state = new_state

    def check_states(self):

        while not rospy.is_shutdown():

            if self.state == self.setup_state:
                rospy.sleep(0.5)
                self.state = self.marker0_state

            elif self.state == self.marker0_state:
                robot_tf = self.tf_buff.lookup_transform('map', 'cf1/base_link', rospy.Time(0))
                dist = point_dist(self.marker0_obs_pose.pose.position,
                                  robot_tf.transform.translation)
                if dist < self.tol:
                    self.state = self.marker1_state
                else:
                    # Actual action in this state
                    self.marker0_obs_pose.header.stamp = rospy.Time.now()
                    self.goal_pub.publish(self.marker0_obs_pose)

            elif self.state == self.marker1_state:
                robot_tf = self.tf_buff.lookup_transform('map', 'cf1/base_link', rospy.Time(0))
                dist = point_dist(self.marker1_obs_pose.pose.position,
                                  robot_tf.transform.translation)
                if dist < self.tol:
                    self.state = self.marker2_state
                else:
                    # Actual action in this state
                    self.marker1_obs_pose.header.stamp = rospy.Time.now()
                    self.goal_pub.publish(self.marker1_obs_pose)

            elif self.state == self.marker2_state:
                robot_tf = self.tf_buff.lookup_transform('map', 'cf1/base_link', rospy.Time(0))
                dist = point_dist(self.marker2_obs_pose.pose.position,
                                  robot_tf.transform.translation)
                if dist < self.tol:
                    #self.state = self.stopsign_state
                    pass
                else:
                    # Actual action in this state
                    self.marker2_obs_pose.header.stamp = rospy.Time.now()
                    self.goal_pub.publish(self.marker2_obs_pose)

            elif self.state == self.stopsign_state:
                robot_tf = self.tf_buff.lookup_transform('map', 'cf1/base_link', rospy.Time(0))
                dist = point_dist(self.marker0_obs_pose.pose.position,
                                  robot_tf.transform.translation)
                if dist < self.tol:
                    self.state = self.done_state
                else:
                    # Actual action in this state
                    self.stopsign_obs_pose.header.stamp = rospy.Time.now()
                    self.goal_pub.publish(self.stopsign_obs_pose)

            # Here for completness sake
            elif self.state == self.done_state:
                pass

            self.rate.sleep()

if __name__ == '__main__':
    sm = StateMachine()
    try:
        sm.check_states()
    except ROSInterruptException:
        pass
