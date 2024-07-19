#!/usr/bin/env python3

import os
import rclpy
from rclpy.node import Node
import py_trees
import py_trees_ros
from ament_index_python.packages import get_package_share_directory
from rclpy.executors import MultiThreadedExecutor
default_location_file = os.path.join(
    get_package_share_directory("mm_bt"),
    "config", "locations.yaml")

import py_trees
import py_trees.display
from std_msgs.msg import *
from behaviours_condition_simulate import *
from behaviours_action_simulate import *
from msg_interface.srv import GenBT

from rclpy.task import Future
import threading
import json

tree_status = None
num_action_max = 7

class AutonomyBehavior(Node):
    def __init__(self,individual,num_action):

        super().__init__("autonomy_node6")
        
        # Create and setup the behavior tree
        self.tree_string = individual
        self.tree = self.create_behavior_tree()
        self.executor_tree = MultiThreadedExecutor()
        self._lock = threading.Lock()
        self.future = []
        self.hold_handle_sub = self.create_subscription(String, "/hold_handle_status", self.hold_handle_callback, 10)
        self.hold_handle_status = ""
        self.result = 1

        self.door_traversal_status_sub = self.create_subscription(String, "/door_traversal_status", self.door_traversal_callback, 10)
        self.door_traversal = ""

        self.behavior_status = String()
        self.behavior_status_sub = self.create_subscription(String,'/behavior_status', self.behavior_status_callback, 10)
        self.action_num = Int32()
        self.action_num = num_action

        self.find_aruco_pub = self.create_publisher(String,'is_find_aruco',10)

        self.gripper_status_sub = self.create_subscription(String,'gripper_status',self.gripper_status_callback,10)
        self.robot_pose_sub = self.create_subscription(Float32MultiArray,'robot_pose',self.robot_pose_callback,10)
        self.gripper_base_pose_sub = self.create_subscription(Float32MultiArray,'gripper_base_pose',self.gripper_base_pose_callback,10)
        self.find_aruco = "False"
        self.has_door_path = "False"
        self.gripper_status = "Open"
        self.robot_pose = [-2.0, -1.5, -1.8]
        self.gripper_base_pose = [0.4,0.1,1.0,1.5,0.0,0.0]

        self.door_status_sub = self.create_subscription(String,'door_status',self.door_status_callback, 10)
        self.door_status = "close"

#---------------------------------- Design BT ----------------------------------#

    def create_behavior_tree(self):
        """ Create behavior tree by picking a next location from a queue """

        initial = ['s(',
                        'f(',
                            {"command":"isGripper", "name":"0", "state":"Open"},
                            {"command":"OpenGripper"},
                        ')',
                ]
        
        individual = []
        individual = initial+self.tree_string+[')']
        print("individual:",individual)

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
#---------------------------------- Create BT ----------------------------------#

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
        return node

    def get_node_from_string(self, string, ros):
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
                node = CreateDoorPath(name = "CreateDoorPath"+string["name"], id=string["id"] ,node= ros) 
            # elif command == "RotateForBetterView":
            #     node = RotateForBetterView(name = "RotateForBetterView", id=string["id"] ,node= ros) 
            elif command == "MMApproachDoor":
                node = MMApproachDoor(name = "MMApproachDoor", taskspace=None ,node= ros) 
            elif command == "MMMoveJoint":
                node = MMMoveJoint(name = "MMMoveJoint" + string["name"], joint=string["joint"] ,node= ros) 
            elif command == "MMMoveLinear":
                node = MMMoveLinear(name = "MMMoveLinear" + string["name"], taskspace=None, node= ros) 
            elif command == "MMFollowTrajectory":
                node = MMFollowTrajectory(name = "MMFollowTrajectory"+string["name"], node= ros) 
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

    def execute(self, period=0.01):
        """ Executes the behavior tree at the specified period. """
        self.tree.tick_tock(period_ms=period*1000.0)
        
        '''especially for hold handle subtask'''
        find_aruco = "True"
        find_aruco_string = String()
        find_aruco_string.data = find_aruco
        self.find_aruco_pub.publish(find_aruco_string)

        future = self.spin_future()
        rclpy.spin_until_future_complete(self.tree.node,future)
        return self.behavior_status, self.door_traversal, self.hold_handle_status, self.door_status, self.gripper_status, self.robot_pose, self.gripper_base_pose
        
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

    def door_traversal_callback(self, msg):
        print("door_traversal:",msg.data)
        self.door_traversal = msg.data
        print("action_num:",self.action_num.data)
        if self.door_traversal == "FAILURE" and self.action_num.data <= num_action_max:
            self.future.set_result(self.result)
            print("Do door_traversal_callback!!")
    
    def behavior_status_callback(self, msg):
        self.behavior_status = msg
        print("action_num:",self.action_num.data)
        if self.behavior_status.data == "FAILURE" or self.door_traversal == "SUCCESS" \
            or (self.action_num.data == num_action_max and self.door_traversal == "FAILURE"): # or (self.action_num.data == num_action_max and self.hold_handle_status == "FAILURE")
            self.future.set_result(self.result)
            self.get_create_path = False
            print("Do behavior_status_callback!!")

    def get_create_path_callback(self, msg):
        self.get_create_path = msg.data

    def gripper_status_callback(self, msg):
        print("BT gripper_status callback:",msg.data)
        self.gripper_status = msg.data

    def robot_pose_callback(self, msg):
        print("BT robot_pose callback:",msg.data)
        self.robot_pose = msg.data

    def gripper_base_pose_callback(self, msg):
        print("BT gripper_base_pose callback:",msg.data)
        self.gripper_base_pose = msg.data

    def hold_handle_callback(self, msg):
        print("hold_handle_status:",msg.data)
        self.hold_handle_status = msg.data

    def door_status_callback(self,msg):
        self.door_status = msg.data

class GenBehaviorTree(Node):
    def __init__(self):
        super().__init__("gen_bahavior_tree")
        # Create and setup the behavior tree
        self.gen_bt_server = self.create_service(GenBT,"/gen_bt", self.gen_bt_callback)
        self.timer = self.create_timer(0.1, self.timer_callback)
        self.req = String()
        self.gen_toggle = False
        # self.hold_handle_sub = self.create_subscription(String, "/hold_handle_status", self.hold_handle_callback, 10)
        # self.hold_handle_status = ""
        self.start_node = False
        self.behavior = None
        self.tree = None
        self.ready_pub = self.create_publisher(Bool, "/ready", 10)
        self.start = False

        self.iteration = 0
        self.iteration_pub = self.create_publisher(Int32, "/iteration", 10)

        self.behavior_status_pub = self.create_publisher(String,'/behavior_status', 10)
        self.behavior_data = String()
        self.hold_handle_pub = self.create_publisher(String, "/hold_handle_status", 10)
        self.hold_handle_status = "FAILURE"
        self.get_topic_toggle = False

        self.pub_toggle_sub = self.create_subscription(Bool, "/pub_toggle", self.pub_toggle_callback, 10)
        self.pub_toggle = False
        self.pub_toggle_pub = self.create_publisher(Bool, "/pub_toggle", 10)
        self.num_action = Int32()

        self.count_check = 0

        # self.find_aruco_pub = self.create_publisher(String,'is_find_aruco',10)
        # self.has_door_path_pub = self.create_publisher(String,'has_door_path',10)
        self.gripper_status_pub = self.create_publisher(String,'gripper_status',10)
        self.robot_pose_pub = self.create_publisher(Float32MultiArray,'robot_pose',10)
        self.gripper_base_pose_pub = self.create_publisher(Float32MultiArray,'gripper_base_pose',10)

        self.find_aruco = "False"
        self.has_door_path = "False"
        self.gripper_status = "Open"
        self.robot_pose = []
        self.gripper_base_pose = []

        self.door_traversal_status_pub = self.create_publisher(String, "/door_traversal_status", 10)
        self.door_traversal = "FAILURE"
        self.door_status_pub = self.create_publisher(String,'door_status', 10)
        self.door_status = "close"

    def timer_callback(self):
        if self.gen_toggle:
            self.start = True
            self.generator()
            self.count_check += 0

        self.count_check += 1

        if self.get_topic_toggle:
            if self.behavior_data.data == '':
                self.behavior_data.data = 'FAILURE'
            self.behavior_status_pub.publish(self.behavior_data)
            door_traversal_status = String()
            door_traversal_status.data = self.door_traversal
            if self.door_traversal == '':
                door_traversal_status.data = "FAILURE"
            self.door_traversal_status_pub.publish(door_traversal_status)

            if self.hold_handle_status == '':
                self.hold_handle_status = 'FAILURE'
            self.hold_handle_pub.publish(String(data=self.hold_handle_status))
            self.door_status_pub.publish(String(data=self.door_status))
            self.gripper_status_pub.publish(String(data=self.gripper_status))

            robot_pose = Float32MultiArray()
            robot_pose.data = self.robot_poses
            self.robot_pose_pub.publish(robot_pose)
            
            gripper_base_pose = Float32MultiArray()
            gripper_base_pose.data = self.gripper_base_pose
            self.gripper_base_pose_pub.publish(gripper_base_pose)

            if self.pub_toggle:
                self.get_logger().info("Get Pub_toggle: True!!")
                self.get_topic_toggle = False
                self.pub_toggle_pub.publish(Bool(data=False))

            self.count_check = 0
        
        if self.count_check >= 50:
            self.behavior_status_pub.publish(String(data='FAILURE'))
            self.door_traversal_status_pub.publish(String(data='FAILURE'))
            self.count_check = 0
            if self.pub_toggle:
                self.get_topic_toggle = False
                self.pub_toggle_pub.publish(Bool(data=False))

    def gen_bt_callback(self, request, response):
        self.gen_toggle = True
        self.req = request.generate_bt
        self.num_action = request.action_num
        self.start_node = True
        tree_string = self.req.data
        tree_json = json.loads(tree_string)
        self.tree = tree_json["key"]
        print("Receive gen_bt request successfully!!!")

        return response
    
    def generator(self):

        if self.start_node == True:
            self.behavior = AutonomyBehavior(self.tree,self.num_action)
            print("Create autonomy_node6 SUCCESS")
            self.start_node = False

        ready = Bool()
        ready.data = False
        self.ready_pub.publish(ready)
        self.behavior_data, self.door_traversal, self.hold_handle_status, self.door_status, self.gripper_status, self.robot_pose, self.gripper_base_pose = self.behavior.execute()
        print("door_traversal_status is:",self.door_traversal)
        print("behavior_data is:",self.behavior_data)
        self.get_topic_toggle = True
        print("Exit execute bt!!")
        self.behavior.destroy_node()
        print("autonomy_node6 destroied")
        self.gen_toggle = False
        ready.data = True
        self.ready_pub.publish(ready)

    def pub_toggle_callback(self, msg):
        self.pub_toggle = msg.data
        
if __name__=="__main__":   
    rclpy.init()
    gen_behavior = GenBehaviorTree()
    rclpy.spin(gen_behavior)  
    gen_behavior.destroy_node()
    rclpy.shutdown()