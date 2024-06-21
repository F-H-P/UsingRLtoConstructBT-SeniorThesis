
import py_trees
# import py_trees_ros
# import rcl_interfaces.msg as rcl_msgs
# import rcl_interfaces.srv as rcl_srvs
# import transforms3d
# import rclpy
# import std_msgs.msg as std_msgs
# import geometry_msgs.msg as geometry_msgs
# import nav_msgs.msg as nav_msgs
# import rclpy.qos
# import sensor_msgs.msg as sensor_msgs
# from sensor_msgs.msg import LaserScan
# import numbers
import time
# from typing import Any, Callable
# import random, math
# import numpy as np
# from rclpy.action import ActionClient
from nav2_msgs.action import *
############################

# from py_trees_ros import subscribers
# from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
# from control_msgs.msg import DynamicJointState
from std_msgs.msg import Float32MultiArray # , Float64
# from geometry_msgs.msg import Twist, PoseWithCovarianceStamped, PoseArray, Pose, PoseStamped, Quaternion, Point
# from ros2_aruco_interfaces.msg import ArucoMarkers
# from ros2_aruco.ros2_aruco_interfaces.msg import ArucoMarkers
# import tf2_ros
# from tf2_ros import TransformBroadcaster
# from action_msgs.msg import GoalStatus
# from tf2_ros import TransformException
# from tf2_ros.buffer import Buffer
# from tf2_ros.transform_listener import TransformListener
from std_msgs.msg import String #, Float64MultiArray
# import random

class isPathExists(py_trees.behaviour.Behaviour):
    def __init__(self, name, node, pose):
        """Initialise with a behaviour name."""
        super(isPathExists, self).__init__(name)
        self.pose = pose
        self.node = node
        self.blackboard = py_trees.blackboard.Client(name="Global")
        self.name = name
        self.blackboard.register_key(key=self.name, access=py_trees.common.Access.WRITE)
        self.blackboard.set(self.name, None)

        self.door_status_sub = self.node.create_subscription(String,'door_status',self.door_status_callback, 10)
        self.door_status = None
    
    def initialise(self) -> None:
        if self.blackboard.get(self.name):
            return py_trees.common.Status.SUCCESS
        elif self.blackboard.get(self.name) == False:
            return py_trees.common.Status.FAILURE
        
    def door_status_callback(self, msg):
        self.door_status = msg.data

    def update(self) -> py_trees.common.Status:
        if self.door_status == "open":
            return py_trees.common.Status.SUCCESS
        elif self.door_status == "close":
            return py_trees.common.Status.FAILURE
        return py_trees.common.Status.RUNNING

    def terminate(self, new_status: py_trees.common.Status) -> None:
        self.logger.info(f"Terminated with status {new_status}")

class isFindMarker(py_trees.behaviour.Behaviour):
    def __init__(self, name, node, id):
        """Initialise with a behaviour name."""
        super(isFindMarker, self).__init__(name)
        self.node = node
        self.name = name
        self.blackboard = py_trees.blackboard.Client(name="Global")
        self.blackboard.register_key(key=name, access=py_trees.common.Access.WRITE)
        self.blackboard.set(name, False)

        self.robot_pose_pub = self.node.create_publisher(Float32MultiArray,'robot_pose', 10)
        self.robot_pose = Float32MultiArray()
        self.robot_pose.data = [-2.0,-1.5,-1.8]
        self.gripper_base_pose_sub = self.node.create_subscription(Float32MultiArray,'/gripper_base_pose',self.gripper_base_pose_callback, 10)
        self.gripper_base_pose = Float32MultiArray()
        self.gripper_base_pose.data = [0.4,0.1,1.0,1.5,0.0,0.0]
        self.robot_pose_aruco_sub = self.node.create_subscription(Float32MultiArray,'robot_pose_aruco',self.robot_pose_aruco_callback,10)
        self.robot_pose_aruco = None

        self.find_aruco_pub = self.node.create_publisher(String,'is_find_aruco', 10)
        self.find_aruco = ""
        
        self.failure_tick = 0
        self.hold_handle_status =String()
        self.hold_handle_pub = self.node.create_publisher(String,'/hold_handle_status', 10)
        self.behavior_status = String()
        self.behavior_status_pub = self.node.create_publisher(String,'/behavior_status', 10)

    def initialise(self) -> None:
        """Backup and set a new context."""
        if self.blackboard.get(self.name):
            return py_trees.common.Status.SUCCESS
        self.logger.debug("%s.initialise()[switch context]" % (self.__class__.__name__))
        self.start_time = self.node.get_clock().now()
        self.t = time.time()
        self.pre_time = None
        # if self.blackboard.get(self.name):
        #     return py_trees.common.Status.SUCCESS
    # def robot_pose_callback(self, msg):
    #     self.robot_pose = msg
    
    def robot_pose_aruco_callback(self, msg):
        self.robot_pose_aruco = msg.data

    def gripper_base_pose_callback(self, msg):
        self.gripper_base_pose = msg

    def update(self) -> py_trees.common.Status:
        if self.blackboard.get(self.name):
            self.robot_pose_pub.publish(self.robot_pose)
            self.find_aruco_pub.publish(String(data=self.find_aruco))
            self.behavior_status_pub.publish(String(data="Success"))
            return py_trees.common.Status.SUCCESS
        if self.gripper_base_pose.data != [0.0,0.0,0.0,0.0,0.0,0.0]:
            gripper_x = self.gripper_base_pose.data[0]
            gripper_y = self.gripper_base_pose.data[1]
            gripper_z = self.gripper_base_pose.data[2]
            gripper_R = self.gripper_base_pose.data[3]
            gripper_P = self.gripper_base_pose.data[4]
            gripper_Y = self.gripper_base_pose.data[5]

        # print("robot_pose_aruco:", self.robot_pose_aruco)
        if self.robot_pose_aruco != None:
            self.node.get_logger().info("robot_pose_aruco:"+str(self.robot_pose_aruco))
            if round(float(self.robot_pose_aruco[0]),1) == -2.0 and round(float(self.robot_pose_aruco[1]),1) == -1.5 and round(float(self.robot_pose_aruco[2]),1) >= -1.9 and round(float(self.robot_pose_aruco[2]),1) <= -1.7:
                self.node.get_logger().info("Do1!")
                if round(float(gripper_x),1) == 0.4 and round(float(gripper_y),1) == 0.1 and round(float(gripper_z),1) == 1.0 and round(float(gripper_R),1) == 1.5 and round(float(gripper_P),1) == 0.0 and round(gripper_Y,1) == 0.0:
                    # random_num = random.random()
                    self.node.get_logger().info("Do2!")
                    # print("random_num:", random_num)
                    # if random_num <= 0.7:
                    self.blackboard.set(self.name, True)
                    self.find_aruco = "True"
                    self.find_aruco_pub.publish(String(data=self.find_aruco))
                    self.behavior_status.data = "Success"
                    self.behavior_status_pub.publish(self.behavior_status)
                    self.robot_pose_pub.publish(self.robot_pose)
                    return py_trees.common.Status.SUCCESS
                    # else:
                    #     self.failure_tick += 1
            else:
                self.failure_tick += 1
        else:
            self.failure_tick += 1

        if self.failure_tick >= 200:
            self.node.get_logger().info("Failure find aruco tick: "+str(self.failure_tick))
            self.behavior_status.data = "Success"
            self.behavior_status_pub.publish(self.behavior_status)
            self.find_aruco = "False"
            self.find_aruco_pub.publish(String(data=self.find_aruco))
            return py_trees.common.Status.SUCCESS
        
        # return py_trees.common.Status.RUNNING

    def terminate(self, new_status: py_trees.common.Status) -> None:
        
        self.logger.info(f"Terminated with status {new_status}")
        self.client = None

class isGripper(py_trees.behaviour.Behaviour):
    def __init__(self, name, node, state):
        """Initialise with a behaviour name."""
        super(isGripper, self).__init__(name)
        self.node = node
        self.name = name
        self.state = state
        self.blackboard = py_trees.blackboard.Client(name="Global")
        self.blackboard.register_key(key=name, access=py_trees.common.Access.WRITE)
        self.blackboard.set(name, False)

        self.gripper_status_sub = self.node.create_subscription(String,'gripper_status',self.gripper_status_callback, 10)
        self.gripper_status = None

        self.behavior_status = String()
        self.behavior_status_pub = self.node.create_publisher(String,'/behavior_status', 10)
    
    def initialise(self) -> None:
        """Backup and set a new context."""
        self.logger.debug("%s.initialise()[switch context]" % (self.__class__.__name__))

    def gripper_status_callback(self, msg):
        self.gripper_status = msg.data

    def update(self) -> py_trees.common.Status:
        if self.blackboard.get(self.name):
            return py_trees.common.Status.SUCCESS        

        if self.gripper_status == self.state:
            self.blackboard.set(self.name, True)
            self.behavior_status.data = "Success"
            self.behavior_status_pub.publish(self.behavior_status)
            return py_trees.common.Status.SUCCESS
        else:
            # self.behavior_status.data = "Success"
            # self.behavior_status_pub.publish(self.behavior_status)
            return py_trees.common.Status.FAILURE

    def terminate(self, new_status: py_trees.common.Status) -> None: 
        self.logger.info(f"Terminated with status {new_status}")

class isHoldHandle(py_trees.behaviour.Behaviour):   
    # SUCCESS : Gripper is closed, Gripper frame in near handle frame
    def __init__(self, name, node):
        super(isHoldHandle, self).__init__(name)
        self.node = node
        self.name = name
        self.blackboard = py_trees.blackboard.Client(name="Global")
        self.blackboard.register_key(key=name, access=py_trees.common.Access.WRITE)
        self.blackboard.set(name, False)

        self.gripper_status_sub = self.node.create_subscription(String,'gripper_status',self.gripper_status_callback, 10)
        self.gripper_status = ""
        self.gripper_base_pose_sub = self.node.create_subscription(Float32MultiArray,'/gripper_base_pose',self.gripper_base_pose_callback, 10)
        self.gripper_pose = Float32MultiArray()
        self.gripper_pose.data = [-1.0,0.0,0.0,0.0,0.0,0.0]
        self.hold_handle_status_pub = self.node.create_publisher(String, "/hold_handle_status", 10)
        self.hold_handle_status = String()
        self.hold_handle_status.data = "FAILURE"
        self.door_path = None
        self.have_door_path = True
        self.failure_tick = 0

        self.do_gripper_status_callback = False

        self.behavior_status_pub = self.node.create_publisher(String,'/behavior_status', 10)

        # self.do_gripper_status_callback = False
        # self.do_gripper_pose_callback = False

    def initialise(self):
        """ Sends the initial navigation action goal """
        # Check if there is a pose available in the blackboard
        try:
            if self.door_path == None:
                door_path = self.blackboard.get("door_path") ## get door path from blackboard
                # print("door_path:", door_path)
                self.door_path = door_path.poses[0]
                # print(self.taskspace)
        except:
            self.have_door_path = False
    
    def gripper_status_callback(self, msg):
        self.gripper_status = msg.data
        self.do_gripper_status_callback = True

    def gripper_base_pose_callback(self, msg):
        self.gripper_pose = msg
        # self.do_gripper_pose_callback = True
    
    def update(self) -> py_trees.common.Status:

        if self.blackboard.get(self.name):
            self.hold_handle_status_pub.publish(self.hold_handle_status)
            self.behavior_status_pub.publish(String(data="Success"))
            return py_trees.common.Status.SUCCESS  
        
        # if self.do_gripper_status_callback and self.do_gripper_pose_callback:
        if self.gripper_pose.data[0] != -1.0 and self.do_gripper_status_callback:
            # if self.gripper_pose.data[3] != 0.0:
            gripper_x = self.gripper_pose.data[0]
            gripper_y = self.gripper_pose.data[1]
            gripper_z = self.gripper_pose.data[2]
            gripper_R = self.gripper_pose.data[3]
            gripper_P = self.gripper_pose.data[4]
            gripper_Y = self.gripper_pose.data[5]

            self.do_gripper_status_callback = False
            
            self.logger.info(f"Gripper status: {self.gripper_status}")
            self.logger.info(f"Gripper pose: {self.gripper_pose.data}")
            if self.gripper_status == "Closed" and round(gripper_x,1) == 0.8 and round(gripper_y,1) == 0.2 and round(gripper_z,1) == 1.0 and round(gripper_R,1) == 1.5 and round(gripper_P,1) == 0.0 and round(gripper_Y,1) == 0.2:
                self.blackboard.set(self.name, True)
                self.hold_handle_status.data = "SUCCESS"
                self.hold_handle_status_pub.publish(self.hold_handle_status)
                return py_trees.common.Status.SUCCESS
            else:
                self.hold_handle_status.data = "FAILURE"
                self.hold_handle_status_pub.publish(self.hold_handle_status)
                return py_trees.common.Status.FAILURE
        else:
            self.failure_tick += 1
            if self.failure_tick >= 10:
                self.hold_handle_status.data = "FAILURE"
                self.hold_handle_status_pub.publish(self.hold_handle_status)
                return py_trees.common.Status.FAILURE
            # else:
            #     self.hold_handle_status.data = "FAILURE"
            #     self.hold_handle_status_pub.publish(self.hold_handle_status)
            #     return py_trees.common.Status.FAILURE
    
    def terminate(self, new_status: py_trees.common.Status) -> None:
        self.logger.info(f"Terminated with status {new_status}")

class isCreateDoorPath(py_trees.behaviour.Behaviour):   
    # SUCCESS : Gripper is closed, Gripper frame in near handle frame
    def __init__(self, name, node):
        super(isCreateDoorPath, self).__init__(name)
        self.node = node
        self.name = name
        self.blackboard = py_trees.blackboard.Client(name="Global")
        self.blackboard.register_key(key=name, access=py_trees.common.Access.WRITE)
        self.blackboard.set(name, False)

        self.hold_handle_status_pub = self.node.create_publisher(String, "/hold_handle_status", 10)
        self.hold_handle_status = String()
        self.hold_handle_status.data = "FAILURE"

        self.has_door_path_sub = self.node.create_subscription(String,'has_door_path',self.has_door_path_callback, 10)
        self.have_door_path = True
        self.failure_tick = 0

        self.do_has_door_path_callback = False
    
    def gripper_status_callback(self, msg):
        self.gripper_status = msg.data
        self.do_gripper_status_callback = True

    def gripper_base_pose_callback(self, msg):
        self.gripper_pose = msg
        # self.do_gripper_pose_callback = True
    
    def has_door_path_callback(self, msg):
        if msg.data == "True":
            self.have_door_path = True
        else:
            self.have_door_path = False
        self.do_has_door_path_callback = True
        # print("have_door_path:", self.have_door_path)
        
    def update(self) -> py_trees.common.Status:

        if self.blackboard.get(self.name):
            self.hold_handle_status.data = "SUCCESS"
            self.hold_handle_status_pub.publish(self.hold_handle_status)
            return py_trees.common.Status.SUCCESS  
        self.node.get_logger().info("have_door_path:"+ str(self.have_door_path))
        self.node.get_logger().info("do_has_door_path_callback:"+str(self.do_has_door_path_callback))
        if self.do_has_door_path_callback:
            if self.have_door_path:
                self.blackboard.set(self.name, True)
                self.hold_handle_status.data = "SUCCESS"
                self.hold_handle_status_pub.publish(self.hold_handle_status)
                return py_trees.common.Status.SUCCESS
            else:
                self.hold_handle_status.data = "FAILURE"
                self.hold_handle_status_pub.publish(self.hold_handle_status)
                return py_trees.common.Status.FAILURE
        else:
            self.failure_tick+=1
            self.node.get_logger().info("Failure is create path tick: "+str(self.failure_tick))
            if self.failure_tick >= 20:
                self.hold_handle_status.data = "FAILURE"
                self.hold_handle_status_pub.publish(self.hold_handle_status)
                return py_trees.common.Status.FAILURE
    
    def terminate(self, new_status: py_trees.common.Status) -> None:
        self.logger.info(f"Terminated with status {new_status}")

class isDoorTraversal(py_trees.behaviour.Behaviour):   
    # SUCCESS : Gripper is closed, Gripper frame in near handle frame
    def __init__(self, name, node):
        super(isDoorTraversal, self).__init__(name)
        self.node = node
        self.name = name
        self.blackboard = py_trees.blackboard.Client(name="Global")
        self.blackboard.register_key(key=name, access=py_trees.common.Access.WRITE)
        self.blackboard.set(name, False)

        self.door_traversal_sub = self.node.create_subscription(String, "/door_traversal", self.door_traversal_callback, 10)
        self.door_traversal = 'FAILURE'
        self.do_door_traversal = False
        self.failure_tick = 0

        self.door_traversal_status_pub = self.node.create_publisher(String, "/door_traversal_status", 10)

        self.do_gripper_status_callback = False

    def initialise(self):
        """ Sends the initial navigation action goal """
        pass

    def door_traversal_callback(self, msg):
        self.door_traversal = msg.data
        self.do_door_traversal = True
    
    def update(self) -> py_trees.common.Status:

        if self.blackboard.get(self.name):
            self.door_traversal_status_pub.publish(String(data="SUCCESS"))
            return py_trees.common.Status.SUCCESS  
        
        if self.do_door_traversal:
            if self.door_traversal == "SUCCESS":
                self.blackboard.set(self.name, True)
                self.door_traversal_status_pub.publish(String(data="SUCCESS"))
                return py_trees.common.Status.SUCCESS
            else:
                self.door_traversal_status_pub.publish(String(data="FAILURE"))
                return py_trees.common.Status.FAILURE
        else:
            self.failure_tick += 1

        if self.failure_tick >= 10:
            self.door_traversal_status_pub.publish(String(data="FAILURE"))
            return py_trees.common.Status.FAILURE
    
    def terminate(self, new_status: py_trees.common.Status) -> None:
        self.logger.info(f"Terminated with status {new_status}")
