#!/usr/bin/env python3

"""
Autonomy node for the TurtleBot3.
This script relies on a YAML file of potential navigation locations, 
which is listed as a `location_file` ROS parameter.
Example usage:
  ros2 run tb3_autonomy autonomy_node.py
  ros2 run tb3_autonomy autonomy_node.py --ros-args -p location_file:=/path/to/my/file.yaml
  ros2 run tb3_autonomy autonomy_node.py --ros-args -p tree_type:=queue -p target_color:=green
"""

import os
# import yaml
# import random
import rclpy
from rclpy.node import Node
# import time
import py_trees
import py_trees_ros
# from py_trees.common import OneShotPolicy
from ament_index_python.packages import get_package_share_directory
# from sensor_msgs.msg import LaserScan
from rclpy.executors import MultiThreadedExecutor
# from manipulator import Mobile_Manipulator
# from manipulator_server import Gripper, ManipulatorJointMode
# from navigation_server import NavigationTo, GetLocationFromQueue
# from vision import FindTheDoorNode, CreateDoorPath
default_location_file = os.path.join(
    get_package_share_directory("mm_bt"),
    "config", "locations.yaml")
# import re
#!/usr/bin/env python3
import py_trees
# import transforms3d
# import graphviz
import py_trees.display
from std_msgs.msg import *
# from action_msgs.msg import GoalStatus
# from rclpy.action import ActionClient
# from nav2_msgs.action import NavigateToPose
# from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
# from geometry_msgs.msg import PoseArray
# from control_msgs.msg import DynamicJointState
# from builtin_interfaces.msg import Duration
import numpy as np
# from mm_bt.srv import CheckDoor
# from mm_bt.action import MMControl
from behaviours_condition_simulate import *
from behaviours_action_simulate import *
# from sensor_msgs.msg import Range
# from door_traversal.scripts.robot_controller import RobotController
from rclpy.task import Future
import threading

num_action_max = 5
step_num = 3
class AutonomyBehavior(Node):
    def __init__(self):
        super().__init__("autonomy_node6")
        self.tree = self.create_behavior_tree()
        self.find_aruco_pub = self.create_publisher(String,'is_find_aruco',10)
        self.executor_tree = MultiThreadedExecutor()
        self._lock = threading.Lock()
        self.future = []

        # self.hold_handle_sub = self.create_subscription(String, "/hold_handle_status", self.hold_handle_callback, 10)
        self.hold_handle_status = String()
        self.result = 1

        self.behavior_status = String()
        self.behavior_status_sub = self.create_subscription(String,'/behavior_status', self.behavior_status_callback, 10)
        self.action_num = Int32()
        self.action_num.data = 4

        self.can_create_path_pub = self.create_publisher(Bool,'can_create_path', 10)
        self.get_create_path_sub = self.create_subscription(Bool,'get_create_path',self.get_create_path_callback,10)
        self.can_create_path_sub = self.create_subscription(Bool, "/can_create_path", self.can_create_path_callback, 10)
        self.get_create_path = False
        self.can_create_path = Bool()
        self.can_create_path.data = True

        self.door_traversal_status_sub = self.create_subscription(String, "/door_traversal_status", self.door_traversal_callback, 10)
        self.door_traversal = ""

    def create_behavior_tree(self):
        """ Create behavior tree by picking a next location from a queue """
        individual = [
                        's(',
                            's(',
                                'f(',
                                    {"command":"isGripper", "name":"0", "state":"Open"},
                                    {"command":"OpenGripper"},
                                ')',
                                'f(',
                                    {"command":"isHoldHandle", "name":"0"},
                                    's(',
                                        {"command":"CreateDoorPath", "id":238}, 
                                        {"command":"MMApproachDoor", "id":238}, 
                                        {"command":"MMMoveLinear","name":"0"}, 
                                        {"command":"ClosedGripper"},
                                    ')',
                                ')',
                                {"command":"MMFollowTrajectory", "name":"0"},
                                {"command":"OpenGripper"},
                                {"command":"MMMoveJoint", "name":"0", "joint":[0.0,-1.6,2.3,-0.75,1.57,-1.57]},
                                # {"command":"MMFollowTrajectory", "name":"1"},
                                {"command":"NavigateTo", "name":"Goal" ,"pose":[-1.0, -8.0, -2.1]},

                                # 'sm(',
                                # {"command":"NavigateTo", "name":"Door0" ,"pose":[-1.0, -0.5, -1.1]},
                                # {"command":"NavigateTo", "name":"Door1" ,"pose":[-2.0, -1.5, -2.1]},
                                
                                # ')',
                                # 'f(',
                                    # 's(',
                                    #     {"command":"isPathExists", "name":"isPathExists" ,"pose":[-1.0, -8.0, -2.1]},
                                    #     # {"command":"NavigateTo", "name":"Door2" ,"pose":[-1.0, -8.0, -2.1]},
                                    #     # {"command":"MMMoveJoint", "name":"2" ,"joint":[1.57,-2.3,2.7,-0.5,1.57,-1.57]},
                                    #     # {"command":"MMMoveJoint", "name":"2" ,"joint":[1.3,-2.9,2.6,0.1,1.57,-1.57]}, 
                                    #     {"command":"MMMoveJoint", "name":"2" ,"joint":[1.57,-2.3,2.7,-0.5,1.57,-1.57]},
                                    #     'f(',   # Find marker
                                    #         {"command":"isFindMarker", "name":"1" , "id": 582},
                                    #         {"command":"NavigateTo", "name":"Door4" ,"pose":[-1.0, -8.0, -2.1]},
                                    #     ')',
                                    #     {"command":"NavigationCancelGoal", "name":"1" },
                                    #     {"command":"MobileBaseControl", "name":"1" ,"mode": "stop"},
                                    #     {"command":"CreateDoorPath", "id":582},
                                    #     {"command":"MMMoveLinear","name":"1"},
                                    #     'f(',
                                    #         {"command":"isGripper", "name":"3", "state":"Closed"},
                                    #         {"command":"ClosedGripper"}, 
                                    #     ')',
                                    #     {"command":"MMFollowTrajectory", "condition":1},
                                    #     'f(',
                                    #         {"command":"isGripper", "name":"4", "state":"Open"},
                                    #         {"command":"OpenGripper"}, 
                                    #     ')',
                                    #     {"command":"MMMoveJoint", "name":"3" ,"joint":[0.0,-1.6,2.3,-0.75,1.57,-1.57]}, 
                                    #     {"command":"NavigateTo", "name":"Door5" ,"pose":[-1.0, -8.0, -2.1]},
                                    #     # {"command":"MobileBaseControl", "mode": "stop"},

                                    #     'f(',
                                    #         {"command":"Running"},
                                    #         # {"command":"MobileBaseControl", "mode": "turn_left"},
                                    #     ')',
                                    # ')',
                                    # {"command":"MobileBaseControl", "name":"0", "mode": "turn_left"},
                                    # 's(',
                                        # 's(',
                                            # 'f(',   # Find marker
                                            #     {"command":"isFindMarker", "name":"0" ,"id": 238},
                                            #     's(',
                                                # {"command":"CreateDoorPath", "id":238},
                                                # {"command":"MobileBaseControl", "name":"0", "mode": "turn_left"},
                                            #     ')',
                                            # ')',
                                            # {"command":"MobileBaseControl", "name":"0" ,"mode": "stop"},
                                        # ')',
                                        # {"command":"MobileBaseControl", "name":"0", "mode": "turn_left"},
                                        # {"command":"MobileBaseControl", "name":"0", "mode": "turn_left"},
                                        # {"command":"MobileBaseControl", "name":"0", "mode": "turn_left"},
                                        # {"command":"CreateDoorPath", "id":238},

                                        # {"command":"isCreateDoorPath", "name":"0"},
                                        # {"command":"MMApproachDoor", "id":238},
                                        # {"command":"MMMoveLinear","name":"0"},
                                        
                                        # 'f(',
                                        #     {"command":"isGripper", "name":"1", "state":"Closed"},
                                            # {"command":"ClosedGripper"}, 
                                        # ')',
                                        # {"command":"ClosedGripper"},
                                        # {"command":"isHoldHandle", "name":"0"},
####################################################################################################################################
                                        # {
                                        # "command":"MMFollowTrajectory", "condition":0},
                                        # 'f(',
                                        #     {"command":"isGripper", "name":"2", "state":"Open"},
                                        #     {"command":"OpenGripper"}, 
                                        # ')',
                                        # {"command":"MMMoveJoint", "name":"1" ,"joint":[0.0,-1.6,2.3,-0.75,1.57,-1.57]}, 
                                        # {"command":"NavigateTo", "name":"Door3", "pose":[-2.0, -1.0, -2.1]},
                                        # {"command":"MMMoveJoint", "name":"2" ,"joint":[1.57,-2.3,2.7,-0.5,1.57,-1.57]},
                                        # # {"command":"MMMoveJoint", "name":"2" ,"joint":[1.3,-2.9,2.6,0.1,1.57,-1.57]}, 

                                        # 'f(',   # Find marker
                                        #     {"command":"isFindMarker", "name":"1" , "id": 582},
                                            # {"command":"NavigateTo", "name":"Door4" ,"pose":[-1.0, -8.0, -2.1]},
                                        # ')',
                                        # {"command":"NavigationCancelGoal", "name":"1" },
                                        # {"command":"MobileBaseControl", "name":"1" ,"mode": "stop"},
                                        # {"command":"CreateDoorPath", "id":582},
                                        # {"command":"MMMoveLinear","name":"1"},
                                        # 'f(',
                                        #     {"command":"isGripper", "name":"3", "state":"Closed"},
                                        #     {"command":"ClosedGripper"}, 
                                        # ')',
                                        # {"command":"MMFollowTrajectory", "condition":1},
                                        # 'f(',
                                        #     {"command":"isGripper", "name":"4", "state":"Open"},
                                        #     {"command":"OpenGripper"}, 
                                        # ')',
                                        # {"command":"MMMoveJoint", "name":"3" ,"joint":[0.0,-1.6,2.3,-0.75,1.57,-1.57]}, 
                                        # {"command":"NavigateTo", "name":"Door5" ,"pose":[-1.0, -8.0, -2.1]},
                                        # # {"command":"MobileBaseControl", "mode": "stop"},

                                        # 'f(',
                                        #     {"command":"Running"},
                                        #     # {"command":"MobileBaseControl", "mode": "turn_left"},
                                        # ')',
                                        # {"command":"Running"},
                                        # 'f(',
                                        #     {"command":"isArrived", "name":"0"},
                                            
                                        # ')',
                                        # {"command":"NavigateTo", "name":"1", "pose":[-2.0, -1.0, -2.1]},
                                        # {"command":"MMMoveJoint", "name":"1" ,"joint":[1.3,-2.3,2.7,-0.5,1.57,-1.57]},  
                                        
                                        # {"command":"NavigationToCheckAruco", "pose":[-1.0, -8.0, -2.1], "id":582},
                                        # 
                                        # 'f(',   # Find marker
                                        #     {"command":"isFindMarker", "name":"1" , "id": 238},
                                        #     {"command":"MMMoveJoint", "joint":[0.0,-1.9,2.6,-0.75,1.57,-1.57]},
                                        #     {"command":"MobileBaseControl", "mode": "turn_left"},
                                        # ')',
                                        # 'f(',   # Move Linear
                                        #     {"command":"isAchived", "name":"0"},
                                        #     {"command":"MMMoveLinear"},
                                        # ')',
                                        {"command":"isDoorTraversal"},
                                    ')',
                                    
                                # ')',
                            # ')',
                        ')'
                        
        ]

        has_children = False
        root = None
       
        if root is None:
            root, has_children = self.get_node_from_string(individual[0], None)
            individual.pop(0)
        else:
            has_children = False

        tree = py_trees_ros.trees.BehaviourTree(root, unicode_tree_debug=False)
        py_trees.blackboard.Blackboard.enable_activity_stream(maximum_size=100000)
        
        tree.setup(timeout=15.0, node=self)

        if has_children:
            self.create_from_string(individual, root, tree.node)

        py_trees.display.render_dot_tree(root)
        return tree  

    def create_from_string(self, string, node, ros):
        """
        Recursive function to generate the tree from a string
        """
        while len(string) > 0:
            if string[0] == ')':
                string.pop(0)
                return node

            newnode, has_children = self.get_node_from_string(string[0], ros)
            string.pop(0)
            if has_children:
                #Node is a control node or decorator with children - add subtree via string and then add to parent
                newnode = self.create_from_string(string, newnode, ros)
                node.add_child(newnode)
            else:
                #Node is a leaf/action node - add to parent, then keep looking for siblings
                node.add_child(newnode)

        #This return is only reached if there are too few up nodes
        return node

    def get_node_from_string(self, string, ros):
        # pylint: disable=too-many-branches
        """
        Returns a py trees behavior or composite given the string
        """
        has_children = False
        if (type(string) == dict):
            command = string["command"] 
            if command == "OpenGripper":
                node = Gripper(name = "OpenGripper", command = "Open", node = ros)
            elif command == "ClosedGripper":
                node = Gripper(name = "ClosedGripper", command="Closed" ,node= ros) 
            elif command == "NavigateTo":
                node = NavigationTo(string["name"], string["pose"], ros) 
            # elif command == "IsArrived":
            #     node = IsArrived("IsArrived", string["pose"], ros) 
            elif command == "CreateDoorPath":
                node = CreateDoorPath(name = "CreateDoorPath", id=string["id"] ,node= ros) 
            # elif command == "RotateForBetterView":
            #     node = RotateForBetterView(name = "RotateForBetterView", id=string["id"] ,node= ros) 
            elif command == "MMApproachDoor":
                node = MMApproachDoor(name = "MMApproachDoor", taskspace=None ,node= ros) 
            elif command == "MMMoveJoint":
                node = MMMoveJoint(name = "MMMoveJoint" + string["name"], joint=string["joint"] ,node= ros) 
            elif command == "MMMoveLinear":
                node = MMMoveLinear(name = "MMMoveLinear" + string["name"], taskspace=None, node= ros) 
            elif command == "MMFollowTrajectory":
                node = MMFollowTrajectory(name = "MMFollowTrajectory", node= ros) 
            # elif command == "NavigationCancelGoal":
            #     node = NavigationCancelGoal( name="NavigationCancelGoal", node= ros )
            # elif command == "NavigationToCheckAruco":
            #     node = NavigationToCheckAruco( name="NavigationToCheckAruco", pose=string["pose"], node= ros, id=string["id"])
            elif command == "MobileBaseControl":
                node = MobileBaseControl( name="MobileBaseControl" + string["name"], node=ros, mode=string["mode"] )
            elif command == "tf2bb":
                node = py_trees_ros.transforms.ToBlackboard(name="", variable_name="test", target_frame="base_link", source_frame="map", qos_profile=py_trees_ros.utilities.qos_profile_unlatched())
            elif command == "isFindMarker":
                node = isFindMarker( name="isFindMarker"+string["name"] ,node=ros, id=string["id"])
            # elif command == "isArrived":
            #     node = isArrived( name="isArrived" + string["name"], node=ros)
            # elif command == "isAchived":
            #     node = isAchived( name="isAchived" + string["name"], node=ros)
            elif command == "isGripper":
                node = isGripper( name="isGripper" + string["name"], state = string["state"], node=ros)
            elif command == "isPathExists":
                node = isPathExists( name="isPathExists", pose = string["pose"], node=ros)
            elif command == "isHoldHandle":
                node = isHoldHandle(name = "isHoldHandle",node = ros)
            elif command == "isCreateDoorPath":
                node = isCreateDoorPath(name="isCreateDoorPath", node=ros)
                
            elif command == "isDoorTraversal":
                node = isDoorTraversal(name="isDoorTraversal", node=ros)
                
            # elif command == "Running":
            #     node = Running(name="Running", node=ros, state=False)

        elif string == 'f(':
            node = py_trees.composites.Selector('Fallback', memory=False)
            has_children = True
        elif string == 'fm(':
            node = py_trees.composites.Selector('Fallback', memory=True)
            has_children = True
        elif string == 's(':
            node = py_trees.composites.Sequence('Sequence', memory=False)
            has_children = True
        elif string == 'sm(':
            node = py_trees.composites.Sequence('Sequence', memory=True)
            has_children = True
        elif string == 'p(':
            node = py_trees.composites.Parallel(
                name="Parallel",
                policy=py_trees.common.ParallelPolicy.SuccessOnAll(synchronise=False))
            has_children = True
        elif string == 'nonpytreesbehavior':
            return False
        else:
            raise Exception("Unexpected character", string)
        return node, has_children

    def post_tick_handler(snapshot_visitor, behaviour_tree):
        print(
            py_trees.display.unicode_tree(
                behaviour_tree.root,
                visited=snapshot_visitor.visited,
                previously_visited=snapshot_visitor.visited
            )
        )
        pass

    def execute(self, period=0.01):
        """ Executes the behavior tree at the specified period. """
        self.tree.tick_tock(period_ms=period*1000.0)
        # rclpy.spin_once(self.tree.node, executor=self.executor_tree)
        # rclpy.spin(self.tree.node, executor=self.executor_tree)
        
        '''especially for hold handle subtask'''
        find_aruco = "True"
        find_aruco_string = String()
        find_aruco_string.data = find_aruco
        self.find_aruco_pub.publish(find_aruco_string)

        # print("get create path: ",self.get_create_path)
        # if self.get_create_path == False:
        #     self.can_create_path_pub.publish(self.can_create_path)
        #     print("pub can_create_path!!")
        
        future = self.spin_future()
        rclpy.spin_until_future_complete(self.tree.node,future)

        return self.behavior_status, self.hold_handle_status

    def spin_future(self) -> Future:
        """
        Make a service request and asyncronously get the result.

        :param request: The service request.
        :return: A future that completes when the request does.
        :raises: TypeError if the type of the passed request isn't an instance
          of the Request type of the provided service when the client was
          constructed.
        """
        print("Do spin_future!!")

        with self._lock:
            future = Future()
            self.future = future

        return future
    
    # def hold_handle_callback(self, msg):
    #     # print("hold_handle_status:",msg.data)
    #     self.hold_handle_status = msg.data
    #     if self.hold_handle_status == "FAILURE" and self.action_num.data <= num_action_max:
    #         self.future.set_result(self.result)
    #         # print("Do hold_handle_callback!!")
    def door_traversal_callback(self, msg):
        print("door_traversal:",msg.data)
        self.door_traversal = msg.data
        print("action_num:",self.action_num.data)
        if self.door_traversal == "FAILURE" and self.action_num.data <= num_action_max:
            self.future.set_result(self.result)
            print("Do door_traversal_callback!!")

    def behavior_status_callback(self, msg):
        self.behavior_status = msg
        if self.behavior_status.data == "FAILURE" or self.door_traversal == "SUCCESS" :
            self.future.set_result(self.result)
            # print("Do behavior_status_callback!!")

    def get_create_path_callback(self, msg):
        self.get_create_path = msg.data

    def can_create_path_callback(self, msg):
        self.can_create_path = msg
    
class TestNode(Node):
    def __init__(self):
        super().__init__("test_node")
        self.timer = self.create_timer(1,self.timer_callback)

        self.find_aruco_pub = self.create_publisher(String,'is_find_aruco',10)
        self.find_aruco_sub = self.create_subscription(String,'is_find_aruco',self.find_aruco_callback,10)
        # self.has_door_path_sub = self.create_subscription(String,'has_door_path',self.has_door_path_callback,10)
        self.has_door_path_pub = self.create_publisher(String,'has_door_path',10)
        # self.gripper_status_sub = self.create_subscription(String,'gripper_status',self.gripper_status_callback,10)
        self.gripper_status_pub = self.create_publisher(String,'gripper_status',10)
        self.robot_pose_sub = self.create_subscription(Float32MultiArray,'robot_pose',self.robot_pose_callback,10)
        self.robot_pose_pub = self.create_publisher(Float32MultiArray,'robot_pose',10)
        self.gripper_base_pose_sub = self.create_subscription(Float32MultiArray,'gripper_base_pose',self.gripper_base_pose_callback,10)
        self.gripper_base_pose_pub = self.create_publisher(Float32MultiArray,'gripper_base_pose',10)

        self.behavior_status_sub = self.create_subscription(String,'/behavior_status', self.behavior_status_callback, 10)
        self.behavior_data = String()

        # self.hold_handle_sub = self.create_subscription(String, "/hold_handle_status", self.hold_handle_callback, 10)

        self.can_create_path_pub = self.create_publisher(Bool,'can_create_path', 10)
        
        self.BT_status = "FAILURE"

        self.find_aruco = "True"
        self.has_door_path = "False"
        self.gripper_status = "Open"
        self.robot_pose = []
        self.gripper_base_pose = []
        self.step_num = step_num

        self.do_robot_callback = False
        self.do_gripper_callback = False
        self.do_find_aruco = True

        self.start = True
        self.run_toggle = True

        self.door_status_sub = self.create_subscription(String,'door_status',self.door_status_callback, 10)
        self.door_status = "close"
        self.do_door_status = False

        self.door_traversal_status_sub = self.create_subscription(String, "/door_traversal_status", self.door_traversal_callback, 10)
        self.door_traversal_status_pub = self.create_publisher(String, "/door_traversal_status", 10)
        self.door_traversal = ""
        self.d_old = 0.0

    def timer_callback(self):
        if self.start:
            if self.run_toggle:
                self.run()
                self.run_toggle = False

            if self.do_robot_callback and self.do_gripper_callback and self.do_find_aruco:
                info = self._get_information()
                self.cal_reward(info)
                self.start = False
                self.do_gripper_callback = False
                self.do_robot_callback = False

    def reset_topic(self):
        self.find_aruco = "True" #False
        self.has_door_path = "False"
        self.gripper_status = "Open"
        self.robot_pose = [-2.0, -1.5, -1.8]
        self.gripper_base_pose = [0.4,0.1,1.0,1.5,0.0,0.0]

        find_aruco = String()
        find_aruco.data = self.find_aruco
        self.find_aruco_pub.publish(find_aruco)
        has_door_path = String()
        has_door_path.data = self.has_door_path
        self.has_door_path_pub.publish(has_door_path)
        gripper_status = String()
        gripper_status.data = self.gripper_status
        self.gripper_status_pub.publish(gripper_status)
        robot_pose = Float32MultiArray()
        robot_pose.data = self.robot_pose
        self.robot_pose_pub.publish(robot_pose)
        gripper_base_pose = Float32MultiArray()
        gripper_base_pose.data = self.gripper_base_pose
        self.gripper_base_pose_pub.publish(gripper_base_pose)
        self.can_create_path_pub.publish(Bool(data=True))
        self.get_logger().info("reset_topic success!!")

    def _get_information(self): 
        #def get_info(self, gripper_pose):
        # reture info that are used to compute the reward in dictionary type
        """
        "distance": math.dist(self._agent_location, self._target_location),
        "laser": self._laser_reads,
        "angle": self._theta
        """
        behavior_status = 0
        BT_status = 0
        door_status = 0

        self.get_logger().info("OpenDoorEnv: get_information")
        if self.behavior_data.data == "Success":
            behavior_status = 0
        elif self.behavior_data.data == "FAILURE":
            behavior_status = 1

        if self.BT_status == "SUCCESS":
            BT_status = 1
        elif self.BT_status == "FAILURE":  
            BT_status = 0

        self.get_logger().info("Get info behavior_data"+str(self.behavior_data))
        self.get_logger().info("Get info BT_status"+str(self.BT_status))
        self.get_logger().info("Get info robot_pose"+str(self.robot_pose))
        self.get_logger().info("Get info gripper_base_pose"+str(self.gripper_base_pose))

        target_pose = [-1.0,-8.0,-2.1]
        # gripper_base_pose = [0,0,0]
        # gripper_base_pose[0] = self.robot_pose[0]-self.gripper_base_pose[0]
        # gripper_base_pose[1] = self.robot_pose[1]-self.gripper_base_pose[1]
        # gripper_base_pose[2] = self.robot_pose[2]
        dist_goal_robot = self.cal_distance(target_pose, self.robot_pose)

        # if self.find_aruco == "True":
        #     find_aruco = 1
        # else:
        #     find_aruco = 0
        if self.door_status == "open":
            door_status = 1
        else:
            door_status = 0

        info = {
                    "behavior_status": behavior_status,
                    "BT_status": BT_status,
                    "dist_goal_robot": dist_goal_robot,
                    "door_status": door_status
                }

        self.get_logger().info("OpenDoorEnv: _get_information success!!")
        return info
    
    def cal_reward(self, info):
        self.get_logger().info("OpenDoorEnv: cal_reward")
        behavior_status = info["behavior_status"]
        BT_status = info["BT_status"]
        dist_goal_robot = info["dist_goal_robot"]
        door_status = info["door_status"]
        D = 0.0
        if (round(self.d_old,3)-round(dist_goal_robot,3)) == 0.0:
            D = 1.0

        '''setup for EX1'''
        d_max = 6.59 ####
        d_min = 0.0 ####

        w_d = 20.0
        w_pd = -20.0
        w_s = 100.0
        w_cb = -20.0
        w_n = 10.0

        reward = 0
        '''goal seeking'''
        # print("door status:",door_status,", type:",type(door_status))
        # print("dist_goal_robot:",dist_goal_robot,", type:",type(dist_goal_robot))
        print("door status:",door_status)
        reward_d = w_d*(((dist_goal_robot-d_max)/(d_min-d_max))+door_status)
        # print("reward_d:",reward_d,", type:",type(reward_d))
        reward_pd = w_pd*D
        reward_s = w_s*BT_status
        reward_cb = w_cb*behavior_status
        reward_n = (BT_status*(w_n*math.exp(-((self.step_num-1)/(num_action_max-1))/0.5)))

        reward = reward_d+reward_pd+reward_s+reward_cb+reward_n

        # returns[0]+=reward_d
        # returns[1]+=reward_pd
        # returns[2]+=reward_s
        # returns[3]+=reward_cb
        # returns[4]+=reward_n
        # returns[5]+=reward

        '''------------------------------------------------------------'''
        
        # reward = -(1.5*behavior_status)+(10.0*BT_status)-(5.0*distance_handle_gripper)-1.0 #+(find_aruco*5.0)
        # if reward < 0.0:
        #     reward=reward*(self.step_num/2)
        # else:
        #     reward=reward/(self.step_num/2)

        # self.get_logger().info("behavior status:"+str(behavior_status))
        self.d_old = dist_goal_robot
        self.get_logger().info("reward:"+str(reward))
        return reward

    def behavior_status_callback(self, msg):
        self.behavior_data = msg
        # self.get_logger().info("Behavior_status: "+str(self.behavior_data))
        # self.get_logger().info("Do behavior status callback!!!!")

    # def hold_handle_callback(self, msg):
    #     self.BT_status = msg.data
        # self.get_logger().info("HoldHandle_status: "+str(self.BT_status))
        # self.get_logger().info("Do BT status callback!!!!")

    def door_traversal_callback(self, msg):
        self.BT_status = msg.data
        print("door_traversal:",msg.data)

    def door_status_callback(self,msg):
        self.door_status = msg.data
        self.do_door_status = True
        print("door_status:",msg.data)

    def cal_distance(self, parent_pose, child_pose):
        distance = np.abs(math.sqrt((parent_pose[0] - child_pose[0])**2 + (parent_pose[1] - child_pose[1])**2 + (parent_pose[2] - child_pose[2])**2))
        return distance
    
    # def cal_reward(self, info):
        # self.get_logger().info("OpenDoorEnv: cal_reward")
        # behavior_status = info["behavior_status"]
        # BT_status = info["BT_status"]
        # distance_handle_gripper = info["distance_handle_gripper"]
        # find_aruco = info["find_aruco"]

        # print("behavior_status:",behavior_status)
        # print("BT_status:",BT_status)
        # print("distance_handle_gripper:",distance_handle_gripper)

        # reward = 0
        # print("step num: ",self.step_num)
        # # reward = (-(1.5*behavior_status)+(10.0*BT_status)-(3.0*distance_handle_gripper)-1.0)
        # print("reward before use step_num: ",reward)
        # reward = -(2.5*behavior_status)+(10.0*BT_status)-(3.0*distance_handle_gripper)-1.0+(find_aruco*5.0)
        # if reward < 0.0:
        #     reward=reward*(self.step_num/2)
        # else:
        #     reward=reward/(self.step_num/2)

        # self.get_logger().info("reward:"+str(reward))
        # return reward

    def run(self):
        print("DO RUN!!")
        self.reset_topic()
        run_bt = AutonomyBehavior()
        run_bt.execute()
        run_bt.destroy_node()

    def robot_pose_callback(self, msg):
        self.robot_pose = msg.data
        self.do_robot_callback = True
        # self.get_logger().info("Do robot_pose_callback!!")
    
    def gripper_base_pose_callback(self, msg):
        self.gripper_base_pose = msg.data
        self.do_gripper_callback = True
        # self.get_logger().info("Do gripper_base_pose_callback!!")
    def find_aruco_callback(self,msg):
        self.find_aruco = msg.data
        self.do_find_aruco = True
        # self.get_logger().info("Do find_aruco_callback!!")

if __name__=="__main__":
    rclpy.init()
    test_node = TestNode()
    rclpy.spin(test_node)  
    test_node.destroy_node()
    rclpy.shutdown()