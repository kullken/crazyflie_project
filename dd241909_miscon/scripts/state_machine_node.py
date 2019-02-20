#!/usr/bin/env python

import math

import rospy
import tf2_ros
import tf2_geometry_msgs
from rospy.exceptions import ROSInterruptException
from tf.transformations import quaternion_from_euler
from geometry_msgs.msg import PoseStamped, Point

from state_machine import State, StateMachine, BlackBoard

def point_dist(p1, p2):
    return math.sqrt((p1.x-p2.x)**2 + (p1.y-p2.y)**2 + (p1.z-p2.z)**2)


class SetupState(State):

    name = 'SETUP_STATE'
    outcomes = ('success', )

    def __init__(self):
        super(SetupState, self).__init__()
        
        self.transitions['success'] = 'HOVER_STATE'
     
    def setup(self):
        pass
    
    def execute(self):
        self.board.rate = rospy.Rate(10)    # State machine updates at most at 10 Hz

        # Access ros parameters
        goal_topic = rospy.get_param(rospy.get_name() + '/navgoal_topic')

        # Save publishers to blackboard publishers
        self.board.goal_pub = rospy.Publisher(goal_topic, PoseStamped, queue_size=5)

        # TODO: Verify that critical nodes are online.
        rospy.sleep(0.5)
        return 'success'


class HoverState(State):
    # TODO: Default behaviour should be to hover inplace at least 0.2 m above floor.

    name = 'HOVER_STATE'
    outcomes = ('success', 'running')

    def __init__(self, x, y, z, roll, pitch, yaw, frame_id='map'):
        super(HoverState, self).__init__()

        pose = PoseStamped()
        pose.header.frame_id = frame_id
        pose.pose.position = Point(x=x, y=y, z=z)
        (pose.pose.orientation.x,
         pose.pose.orientation.y,
         pose.pose.orientation.z,
         pose.pose.orientation.w) = quaternion_from_euler(roll, pitch, yaw)
        self.pose = pose

    def setup(self):
        pass

    def execute(self):
        self.pose.header.stamp = rospy.Time.now()
        self.board.goal_pub.publish(self.pose)
        return 'running'
    


if __name__ == '__main__':
    rospy.init_node('state_machine')

    sm = StateMachine('TOP_LEVEL_SM')
    setup_state = SetupState()
    hover_state = HoverState(0, 0, 0.5, 0, 0, 0, frame_id='map')

    sm.add_state(setup_state)
    sm.add_state(hover_state)

    try:
        sm.setup()
        sm.execute()
    except ROSInterruptException:
        pass

    rospy.spin()
