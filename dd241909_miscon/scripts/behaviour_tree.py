#!/usr/bin/env python

import functools

import rospy
import py_trees
import py_trees_ros

from aruco_msgs.msg import MarkerArray

from dd241909_msgs.msg import Trajectory
from dd241909_msgs.msg import NavigateAction, NavigateGoal
from dd241909_msgs.msg import SimpleAction, SimpleGoal

def create_root():

    # Top-level behaviours
    root = py_trees.composites.Parallel('Root')
    priorities = py_trees.composites.Selector('Priorities')
    idle = py_trees.behaviours.Running('Idle')

    # Backup behaviours
    backup = py_trees.composites.Selector('Backup')
    hover = py_trees_ros.actions.ActionClient(
            name='Hover',
            action_namespace=hover_action,
            action_spec=SimpleAction,
            action_goal=SimpleGoal()
    )
    timeout = py_trees.decorators.Timeout(
            child=hover, 
            name='Max 2 sec',
            duration=2.0
    )
    backup_land = py_trees_ros.actions.ActionClient(
            name='Land',
            action_namespace=land_action,
            action_spec=SimpleAction,
            action_goal=SimpleGoal()
    )

    # Navigate behaviour
    navigate = create_navigate()

    # Tree
    root.add_children([priorities])
    priorities.add_children([navigate, backup, idle])
    backup.add_children([timeout, backup_land])

    return root

def create_navigate():

    # Localise behaviour
    takeoff = py_trees_ros.actions.ActionClient(
            name='Takeoff',
            action_namespace=takeoff_action,
            action_spec=SimpleAction,
            action_goal=SimpleGoal()
    )
    rotate = py_trees_ros.actions.ActionClient(
            name='Rotate',
            action_namespace=rotate_action,
            action_spec=SimpleAction,
            action_goal=SimpleGoal()
    )
    wait_for_marker = py_trees_ros.subscribers.WaitForData(
            name='Wait for marker',
            topic_name=aruco_marker_topic,
            topic_type=MarkerArray,
            clearing_policy=py_trees.common.ClearingPolicy.NEVER
    )

    rotate_and_look = py_trees.composites.Parallel(
            name='Rotate and look',
            policy=py_trees.common.ParallelPolicy.SUCCESS_ON_ONE
    )
    rotate_and_look.add_children([rotate, wait_for_marker])

    localise = py_trees.composites.Sequence('Localise')
    localise.add_children([takeoff, rotate_and_look])
    localise_once = py_trees.decorators.OneShot(localise, 'Once')

    # Plan behaviour
    plan_hover = py_trees_ros.actions.ActionClient(
            name='Hover',
            action_namespace=hover_action,
            action_spec=SimpleAction,
            action_goal=SimpleGoal()
    )
    plan = py_trees_ros.actions.ActionClient(
            name='Plan',
            action_namespace=plan_action,
            action_spec=SimpleAction,
            action_goal=SimpleGoal()
    )

    plan_root = py_trees.composites.Parallel(
            name='Wait for plan',
            policy=py_trees.common.ParallelPolicy.SUCCESS_ON_ONE
    )
    plan_root.add_children([plan_hover, plan])
    plan_once = py_trees.decorators.OneShot(plan_root, 'Once')

    # Run course behaviour
    run_course = py_trees_ros.actions.ActionClient(
            name='Run course',
            action_namespace=navigate_action,
            action_spec=NavigateAction,
            action_goal=NavigateGoal(start=True)
    )
    run_course_once = py_trees.decorators.OneShot(run_course, 'Once')

    # Land and idle
    nav_land = py_trees_ros.actions.ActionClient(
            name='Land',
            action_namespace=land_action,
            action_spec=SimpleAction,
            action_goal=SimpleGoal()
    )
    nav_idle = py_trees.behaviours.Running('Idle')

    # Build tree
    navigate = py_trees.composites.Sequence('Navigate')
    #navigate.add_children([localise_once, plan_once, run_course_once, nav_land, nav_idle])
    navigate.add_children([plan_once, run_course_once, nav_land, nav_idle])

    return navigate

def shutdown(tree):
    tree.interrupt()

if __name__ == '__main__':
    rospy.init_node('behaviour_tree')

    # Access ros parameters
    trajectory_topic    = rospy.get_param('~trajectory_topic')
    aruco_marker_topic  = rospy.get_param('~aruco_marker_topic')
    navigate_action     = rospy.get_param(rospy.get_name() + '/navigate_action')
    plan_action         = rospy.get_param('~plan_action')
    takeoff_action      = rospy.get_param('~takeoff_action')
    rotate_action       = rospy.get_param('~rotate_action')
    hover_action        = rospy.get_param(rospy.get_name() + '/hover_action')
    land_action         = rospy.get_param(rospy.get_name() + '/land_action')

    #root = create_root()
    plan = py_trees_ros.actions.ActionClient(
            name='Plan',
            action_namespace=plan_action,
            action_spec=SimpleAction,
            action_goal=SimpleGoal()
    )
    root = py_trees.decorators.OneShot(plan, 'Once')

    tree = py_trees_ros.trees.BehaviourTree(root)
    rospy.on_shutdown(functools.partial(shutdown, tree))
    tree.setup(20)
    tree.tick_tock(200) # 5 Hz

