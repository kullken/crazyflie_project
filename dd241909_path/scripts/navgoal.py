#!/usr/bin/env python

import math
import rospy
import tf2_ros
import tf2_geometry_msgs
from rospy.exceptions import ROSInterruptException
from tf.transformations import euler_from_quaternion
from geometry_msgs.msg import PoseStamped
from crazyflie_driver.msg import Position

# Current goal (global state)
goal = None

def goal_callback(msg):
    global goal
    goal = msg
    rospy.loginfo('New goal set:\n%s', goal)

def publish_cmd(goal):
    goal.header.stamp = rospy.Time.now()
    goal_frame_id = goal.header.frame_id
    odom_frame_id = 'cf1/odom'
    if not tf_buff.can_transform(goal_frame_id, odom_frame_id, goal.header.stamp):
        rospy.logwarn_throttle(5.0, 'cannot transform goal from {} to {}'.format(goal_frame_id, odom_frame_id))
        return

    goal_odom = tf_buff.transform(goal, odom_frame_id)

    roll, pitch, yaw = euler_from_quaternion((goal_odom.pose.orientation.x,
                                              goal_odom.pose.orientation.y,
                                              goal_odom.pose.orientation.z,
                                              goal_odom.pose.orientation.w))

    # NOTE Yaw is in degrees.
    cmd = Position(goal_odom.header,
                   goal_odom.pose.position.x,
                   goal_odom.pose.position.y,
                   goal_odom.pose.position.z,
                   yaw*180.0/math.pi)

    if cmd.z == 0.0:
        cmd.z = 0.4

    pub_cmd.publish(cmd)

rospy.init_node('navgoal', log_level=rospy.WARN)
tfprefix        = rospy.get_param(rospy.get_name() + '/tfprefix')
navgoal_topic   = rospy.get_param(rospy.get_name() + '/navgoal_topic')

sub_goal = rospy.Subscriber(navgoal_topic, PoseStamped, goal_callback)
pub_cmd  = rospy.Publisher('/cf1/cmd_position', Position, queue_size=2)
tf_buff  = tf2_ros.Buffer()
tf_lstn  = tf2_ros.TransformListener(tf_buff)

def main():
    rate = rospy.Rate(20)  # Hz
    while not rospy.is_shutdown():
        if goal:
            publish_cmd(goal)
        rate.sleep()

if __name__ == '__main__':
    try:
        main()
    except ROSInterruptException:
        pass
