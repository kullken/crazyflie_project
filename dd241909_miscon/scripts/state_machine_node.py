#!/usr/bin/env python

import math

import rospy
import tf2_ros
import tf2_geometry_msgs
import actionlib
from rospy.exceptions import ROSInterruptException
from tf.transformations import quaternion_from_euler
from geometry_msgs.msg import PoseStamped, Point, Pose, Transform
from dd241909_msgs.msg import NavigateAction, NavigateGoal

from state_machine import State, StateMachine, BlackBoard


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

def newPoseStamped(x, y, z, roll=0, pitch=0, yaw=0, frame_id='map', stamp=None):
    pose = PoseStamped()
    pose.header.frame_id = frame_id
    pose.pose.position = Point(x=x, y=y, z=z)
    (pose.pose.orientation.x,
     pose.pose.orientation.y,
     pose.pose.orientation.z,
     pose.pose.orientation.w) = quaternion_from_euler(roll, pitch, yaw)

    if not stamp is None:
        pose.header.stamp = stamp

    return pose


class SetupState(State):

    name = 'SETUP_STATE'
    outcomes = ('success', 'running')

    def __init__(self):
        super(SetupState, self).__init__()
     
    def setup(self):
        pass
    
    def execute(self):
        rospy.loginfo(rospy.get_name() + ': Executing {}'.format(self.name))
        self.board.rate = rospy.Rate(10)    # State machine updates at most at 10 Hz

        # Access ros parameters
        goal_topic          = rospy.get_param(rospy.get_name() + '/navgoal_topic')
        #pose_topic          = rospy.get_param(rospy.get_name() + '/base_pose_topic')
        naviagte_action     = rospy.get_param(rospy.get_name() + '/navigate_action')

        # Subscribers
        #self.board.pose_sub = rospy.Subscriber(pose_topic, PoseStamped, self.pose_cb)

        # Publishers
        self.board.goal_pub = rospy.Publisher(goal_topic, PoseStamped, queue_size=5)

        # Action clients
        self.board.nav_client       = actionlib.SimpleActionClient(naviagte_action, NavigateAction)

        # Check that other systems are online
        rospy.loginfo(rospy.get_name() + ': Waiting for {} action server...'.format(naviagte_action))
        self.board.nav_client.wait_for_server()

        # TODO: Verify that all critical systems are online. Until then:
        rospy.sleep(0.5)

        return 'success'

    def pose_cb(self, pose):
        self.board.current_pose = pose


class HoverState(State):
    # TODO: Default behaviour should be to hover inplace at least 0.2 m above floor.

    name = 'HOVER_STATE'
    outcomes = ('success', 'running')

    def __init__(self, pose):
        super(HoverState, self).__init__()
        self.pose = pose

    def setup(self):
        pass

    def execute(self):
        rospy.loginfo_throttle(1, rospy.get_name() + ': Executing {}'.format(self.name))
        self.pose.header.stamp = rospy.Time.now()
        self.board.goal_pub.publish(self.pose)
        return 'running'


class NavState(State):

    name = 'NAV_STATE'
    outcomes = ('success', 'failure', 'running')

    def __init__(self, tolerance=0.05):
        super(NavState, self).__init__()
        self.tol = tolerance

    def setup(self):
        pass

    def execute(self):
        rospy.loginfo(rospy.get_name() + ': Executing {}'.format(self.name))
        
        self.board.nav_client.send_goal(NavigateGoal(start=True))

        while not self.board.nav_client.wait_for_result(rospy.Duration(0.1)):
            # TODO: Check preemption
            self.board.rate.sleep()
        result = self.board.nav_client.get_result()
        if result.success:
            return 'success'
        else:
            return 'failure'



    


if __name__ == '__main__':
    rospy.init_node('state_machine', log_level=rospy.INFO)


    sm = StateMachine('TOP_LEVEL_SM')
    setup_state     = SetupState()
    nav_state       = NavState()
    hover_state     = HoverState(newPoseStamped(2, 2, 0.4, 0, 0, 0, 'map'))

    setup_state.transitions['success']  = 'NAV_STATE'
    nav_state.transitions['success']    = 'HOVER_STATE'

    sm.add_state(setup_state)
    sm.add_state(nav_state)
    sm.add_state(hover_state)

    rospy.loginfo(rospy.get_name() + ': Starting statemachine...')

    try:
        sm.setup()
        sm.execute()
    except ROSInterruptException:
        pass

    rospy.spin()
