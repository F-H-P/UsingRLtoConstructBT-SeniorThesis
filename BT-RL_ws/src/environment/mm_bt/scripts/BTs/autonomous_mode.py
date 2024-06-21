#!/usr/bin/env python3


import os
import yaml
import random
import rclpy
from rclpy.node import Node
import time
import py_trees
import py_trees_ros
from py_trees.common import OneShotPolicy
from ament_index_python.packages import get_package_share_directory
from sensor_msgs.msg import LaserScan
from rclpy.executors import MultiThreadedExecutor

import transforms3d
import graphviz
import py_trees.display
from std_msgs.msg import *
from action_msgs.msg import GoalStatus
from rclpy.action import ActionClient
from nav2_msgs.action import NavigateToPose
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from geometry_msgs.msg import PoseArray
from control_msgs.msg import DynamicJointState
from builtin_interfaces.msg import Duration
import numpy as np
from mm_bt.srv import CheckDoor
from mm_bt.action import MMControl

import sys
sys.path.insert(0,'/content/WASP-CBSS-BT')
from IPython.display import Image
import bt_node.notebook_interface as notebook_interface
import bt_node.behavior_tree as behavior_tree
behavior_tree.load_settings_from_file('/home/kittinook/ur5_ws/src/mm_bt/scripts/BTs/bt_node/BT_SETTINGS.yaml')
from IPython.core.interactiveshell import InteractiveShell
InteractiveShell.ast_node_interactivity = "all"
from pathlib import Path

class AutonomyBehavior(Node):
    def __init__(self):
        super().__init__("autonomy_node")
        
        # Create and setup the behavior tree
        self.create_behavior_tree()

    def create_behavior_tree(self):
        self.tree = self.create_queue_tree()

    def create_queue_tree(self):
        """ Create behavior tree by picking a next location from a queue """

        root = py_trees.composites.Parallel(
            name="Door Transversal",
            # memory=True
            policy=py_trees.common.ParallelPolicy.SuccessOnAll(
                synchronise=False
            )
        )
        tree = py_trees_ros.trees.BehaviourTree(root, unicode_tree_debug=False)
        # tree = py_trees_ros.trees.BehaviourTree(root, unicode_tree_debug=True)
        

        # tree = py_trees_ros.trees.BehaviourTree(root, unicode_tree_debug=True)
        
        individual = ['s(', 'f(', 'carried weight < 5?', \
                          's(', 'move to CHARGE1', 'charge!', ')', ')', \
                    'f(', 'conveyor light < 1?', 's(', 'move to CONVEYOR_LIGHT!', 'idle!', ')']

        environment = notebook_interface.Environment(seed=0, verbose=False)
        
        my_tree = environment.plot_individual('', 'behavior_tree', individual)
        
        tree = environment.create_tree(individual)

        bt = py_trees_ros.trees.BehaviourTree(tree.root, unicode_tree_debug=False)
        py_trees.blackboard.Blackboard.enable_activity_stream(maximum_size=100)
        tree.setup(timeout=15.0, node=self)
        # py_trees.display.render_dot_tree(root)
        print(bt)
        return tree
   
    def execute(self, period=0.01):
        """ Executes the behavior tree at the specified period. """
        self.tree.tick_tock(period_ms=period*1000.0)
        # executor = MultiThreadedExecutor()
        rclpy.spin(self.tree.node)
        rclpy.shutdown()

if __name__=="__main__":
    rclpy.init()
    
    behavior = AutonomyBehavior()
    behavior.execute()