#!/usr/bin/env python

import functools

import rospy
import py_trees
import py_trees_ros

from dd241909_msgs.msg import NavigateAction, NavigateGoal
from dd241909_msgs.msg import SimpleAction, SimpleGoal

def create_root():
    # Access ros parameters
    navigate_action = rospy.get_param(rospy.get_name() + '/navigate_action')
    hover_action    = rospy.get_param(rospy.get_name() + '/hover_action')
    land_action     = rospy.get_param(rospy.get_name() + '/land_action')

    # Top-level behaviours
    root = py_trees.composites.Parallel('Root')
    priorities = py_trees.composites.Selector('Priorities')
    idle = py_trees.behaviours.Running('Idle')

    # Navigate behaviour
    navigate = py_trees.composites.Sequence('Navigate')
    run_course = py_trees_ros.actions.ActionClient(
            name='Run course',
            action_namespace=navigate_action,
            action_spec=NavigateAction,
            action_goal=NavigateGoal(start=True)
    )
    run_course_once = py_trees.decorators.OneShot(run_course, 'Once')
    nav_land = py_trees_ros.actions.ActionClient(
            name='Land',
            action_namespace=land_action,
            action_spec=SimpleAction,
            action_goal=SimpleGoal()
    )
    nav_idle = py_trees.behaviours.Running('Idle')

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
            name='Max 2sec',
            duration=2.0
    )
    backup_land = py_trees_ros.actions.ActionClient(
            name='Land',
            action_namespace=land_action,
            action_spec=SimpleAction,
            action_goal=SimpleGoal()
    )

    # Tree
    root.add_children([priorities])
    priorities.add_children([navigate, backup, idle])
    navigate.add_children([run_course_once, nav_land, nav_idle])
    backup.add_children([timeout, backup_land])

    return root

def shutdown(tree):
    tree.interrupt()

if __name__ == '__main__':
    rospy.init_node('behaviour_tree')
    root = create_root()
    tree = py_trees_ros.trees.BehaviourTree(root)
    rospy.on_shutdown(functools.partial(shutdown, tree))
    tree.setup(20)
    tree.tick_tock(200) # 5 Hz

