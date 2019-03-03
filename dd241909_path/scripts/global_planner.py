#!/usr/bin/env python

import math
import json

import rospy
import tf2_ros
import tf2_geometry_msgs
from nav_msgs.msg import Path
from geometry_msgs.msg import PoseStamped, Point
from tf.transformations import quaternion_from_euler

from octomap import create_map, Vec3

def transform_to_pose(tf):
    pose = PoseStamped()
    pose.header = tf.header
    pose.pose.position.x = tf.transform.translation.x
    pose.pose.position.y = tf.transform.translation.y
    pose.pose.position.z = tf.transform.translation.z
    pose.pose.orientation = tf.transform.rotation
    return pose

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


class GlobalPlanner(object):

    def __init__(self):
        rospy.init_node('global_planner', log_level=rospy.INFO)
        rospy.loginfo(rospy.get_name() + ': Initialising node...')

        # Access ros parameters
        map_file            = rospy.get_param(rospy.get_name() + '/map_file')
        collision_radius    = rospy.get_param(rospy.get_name() + '/collision_radius')
        tfprefix            = rospy.get_param(rospy.get_name() + '/tfprefix')
        trajectory_topic    = rospy.get_param(rospy.get_name() + '/trajectory_topic')
        #plan_path_action    = rospy.get_param(rospy.get_name() + '/plan_path_action')

        # Publishers
        self.traj_pub = rospy.Publisher(trajectory_topic, Path, queue_size=2)

        # Set up tf stuff
        if tfprefix:
            self.base_frame = tfprefix + '/base_link'
        else:
            self.base_frame = 'base_link'
        self.tf_buff = tf2_ros.Buffer()
        self.tf_lstn = tf2_ros.TransformListener(self.tf_buff)

        # Create action server
        # self.server = actionlib.SimpleActionServer(plan_path_action, 
        #                                            PlanGlobalPathAction, 
        #                                            execute_cb=self.plan_cb,
        #                                            auto_start=False
        #                                            )

        # Parse world map
        rospy.loginfo(rospy.get_name() + ': Creating octomap...')
        with open(map_file, 'r') as file:
            data = json.load(file)
        self.map = create_map(data, float(collision_radius))
        rospy.loginfo(rospy.get_name() + ': Octomap created.')

        # Initialisation done, start action server
        # rospy.loginfo(rospy.get_name() + ': Starting {} action server...'.format(plan_path_action))
        # self.server.start()

    def start(self):
        # TODO: Add while not shutdown loop for online replanning

        start_pose = newPoseStamped(0, 0, 0, 0, 0, 0, self.base_frame)
        start_pose = self.tf_buff.transform(start_pose, 'map', rospy.Duration(0.2))
        start_pose.pose.position.z = 0.4
        poses = [start_pose]
        for gate in self.map.gates:
            poses += self.get_gate_poses(gate)

        path = Path()
        path.header.frame_id = 'map'
        path.header.stamp = rospy.Time.now()
        path.poses = poses

        self.traj_pub.publish(path)

    def plan_cb(self, goal):

        poses = [goal.start]
        for gate in self.map.gates:
            poses += self.get_gate_poses(gate)
        poses.append(goal.goal)

        path = Path()
        path.header.frame_id = 'map'
        path.header.stamp = rospy.Time.now()
        path.poses = poses

        result = PlanGlobalPathResult()
        result.path = path

        self.server.set_succeeded(result)

    def get_gate_poses(self, gate):
        """Gets one pose before and one pose after gate."""
        theta = gate['heading'] * 2*math.pi/360
        normal = Vec3(math.cos(theta), math.sin(theta), 0.0)

        point1 = gate['position'] - 0.2*normal
        point2 = gate['position'] + 0.2*normal

        pose1 = PoseStamped()
        pose1.header.frame_id = 'map'
        (pose1.pose.position.x,
         pose1.pose.position.y,
         pose1.pose.position.z) = point1.x, point1.y, point1.z
        (pose1.pose.orientation.x,
         pose1.pose.orientation.y,
         pose1.pose.orientation.z,
         pose1.pose.orientation.w) = quaternion_from_euler(0.0, 0.0, theta)

        pose2 = PoseStamped()
        pose2.header.frame_id = 'map'
        (pose2.pose.position.x,
         pose2.pose.position.y,
         pose2.pose.position.z) = point2.x, point2.y, point2.z
        (pose2.pose.orientation.x,
         pose2.pose.orientation.y,
         pose2.pose.orientation.z,
         pose2.pose.orientation.w) = quaternion_from_euler(0.0, 0.0, theta)

        # rospy.logwarn('Pre-gate pose: {}'.format(pose1))
        # rospy.logwarn('Post-gate pose: {}'.format(pose2))

        return [pose1, pose2]


if __name__ == '__main__':
    planner = GlobalPlanner()
    planner.start()

    rospy.spin()




