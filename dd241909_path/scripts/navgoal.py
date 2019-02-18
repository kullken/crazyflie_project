#!/usr/bin/env python

import math
import rospy
import tf2_ros
import tf2_geometry_msgs
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
    #goal.header.stamp = rospy.Time.now()
    goal_frame_id = goal.header.frame_id
    odom_frame_id = 'cf1/odom'
    if not tf_buf.can_transform(goal_frame_id, odom_frame_id, goal.header.stamp):
        rospy.logwarn_throttle(5.0, 'cannot transform goal from {} to {}/odom'.format(goal_frame_id, odom_frame_id))
        return

    goal_odom = tf_buf.transform(goal, odom_frame_id)

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

rospy.init_node('navgoal')
sub_goal = rospy.Subscriber('/cf1/move_base_simple/goal', PoseStamped, goal_callback)
pub_cmd  = rospy.Publisher('/cf1/cmd_position', Position, queue_size=2)
tf_buf   = tf2_ros.Buffer()
tf_lstn  = tf2_ros.TransformListener(tf_buf)

def main():
    rate = rospy.Rate(10)  # Hz
    while not rospy.is_shutdown():
        if goal:
            publish_cmd(goal)
        rate.sleep()

if __name__ == '__main__':
    main()
