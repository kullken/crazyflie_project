#!/usr/bin/env python

import math
import rospy
import tf2_ros
import tf2_geometry_msgs
from rospy.exceptions import ROSInterruptException
from tf.transformations import euler_from_quaternion
from geometry_msgs.msg import PoseStamped
from crazyflie_driver.msg import Position


class Navgoal(object):

    def __init__(self):
        rospy.init_node('navgoal', log_level=rospy.INFO)
        rospy.loginfo(rospy.get_name() + ': Initialising node...')
        
        self.navgoal = None
        self.rate = rospy.Rate(20)

        # Access ros parameters
        tfprefix        = rospy.get_param(rospy.get_name() + '/tfprefix')
        navgoal_topic   = rospy.get_param(rospy.get_name() + '/navgoal_topic')
        cmdpos_topic    = rospy.get_param(rospy.get_name() + '/cmdpos_topic')

        # Subscribers
        self.sub_goal = rospy.Subscriber(navgoal_topic, PoseStamped, self.navgoal_cb)

        # Publishers
        self.pub_cmd  = rospy.Publisher(cmdpos_topic, Position, queue_size=1)

        # Set up tf stuff
        if tfprefix:
            self.odom_frame = tfprefix + '/odom'
        else:
            self.odom_frame = 'odom'
        self.tf_buff = tf2_ros.Buffer()
        self.tf_lstn = tf2_ros.TransformListener(self.tf_buff)
    
    def navgoal_cb(self, msg):
        self.navgoal = msg

    def start(self):
        while not rospy.is_shutdown():
            if self.navgoal:
                self.publish_cmd(self.navgoal)
            self.rate.sleep()

    def publish_cmd(self, goal):
        # Crazyflie wants commands in the odometry frame
        goal.header.stamp = rospy.Time.now()
        goal_frame_id = goal.header.frame_id
        if not self.tf_buff.can_transform(goal_frame_id, self.odom_frame, goal.header.stamp):
            rospy.logwarn_throttle(5.0, 'cannot transform goal from {} to {}'.format(goal_frame_id, self.odom_frame))
            return
        goal = self.tf_buff.transform(goal, self.odom_frame)

        # Convert to crazyflie position msg (x,y,z,yaw)
        roll, pitch, yaw = euler_from_quaternion((
                goal.pose.orientation.x,
                goal.pose.orientation.y,
                goal.pose.orientation.z,
                goal.pose.orientation.w
        ))
        # NOTE Yaw is in degrees.
        cmd = Position(
                goal.header,
                goal.pose.position.x,
                goal.pose.position.y,
                goal.pose.position.z,
                yaw*180.0/math.pi
        )
        self.pub_cmd.publish(cmd)


if __name__ == '__main__':
    try:
        ng = Navgoal()
        ng.start()
    except ROSInterruptException:
        pass
