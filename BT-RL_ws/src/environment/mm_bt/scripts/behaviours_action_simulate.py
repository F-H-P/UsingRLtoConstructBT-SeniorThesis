import py_trees
# import transforms3d
# import rclpy
# from action_msgs.msg import GoalStatus
# from rclpy.action import ActionClient
# from nav2_msgs.action import NavigateToPose
# from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
# from control_msgs.msg import DynamicJointState
# from builtin_interfaces.msg import Duration
# from geometry_msgs.msg import Twist
# from nav_msgs.msg import Path
# import tf2_ros
# import numpy as np
import math
# import transformers as tf_trans
# from geometry_msgs.msg import Twist, PoseWithCovarianceStamped, PoseArray, Pose, PoseStamped, Quaternion, Point
from sensor_msgs.msg import *
from std_msgs.msg import *
# from ros2_aruco_interfaces.msg import ArucoMarkers
import py_trees
# import py_trees_ros.trees
# import pycapacity.robot as capacity # robot capacity module
# from visualization_msgs.msg import Marker
# import numpy as np
# from roboticstoolbox.robot.ET import ET
# from roboticstoolbox.robot.ERobot import ERobot
# from roboticstoolbox.robot.Link import Link
# from nav_msgs.msg import Path, Odometry
# from geometry_msgs.msg import PoseStamped, TransformStamped, Twist, Quaternion, Point
# from tf2_ros import TransformBroadcaster
import time
# import transformations as tf
# import json
# from mm_bt.srv import CheckDoor
# from mm_bt.action import MMControl, MoveManipulatorToTarget, MMAdjustBase
# from nav2_simple_commander.robot_navigator import BasicNavigator
from sensor_msgs.msg import *

class CreateDoorPath(py_trees.behaviour.Behaviour): ## 1
    def __init__(self, name, node, id):
        """Initialise with a behaviour name."""
        super(CreateDoorPath, self).__init__(name)
        self.node = node
        self.name = name
        self.blackboard = py_trees.blackboard.Client(name="Global")
        self.blackboard.register_key(key=self.name, access=py_trees.common.Access.WRITE)
        self.blackboard.register_key(key="door_path", access=py_trees.common.Access.WRITE)

        self.find_aruco_toggle = self.node.create_subscription(String,'is_find_aruco',self.find_aruco_callback, 10) ## topic for check 'find aruco'
        self.blackboard.set(self.name, False)
        self.pub_has_door_path = self.node.create_publisher(String, 'has_door_path', 10)
        self.is_find_marker = False

        self.failure_tick = 0
        self.hold_handle_status =String()
        # self.hold_handle_pub = self.node.create_publisher(String,'/hold_handle_status', 10)
        self.behavior_status = String()
        self.behavior_status_pub = self.node.create_publisher(String,'/behavior_status', 10)
        self.logger.info("Do create door path!!")

        # self.can_create_path_sub = self.node.create_subscription(Bool,'can_create_path',self.can_create_path_callback, 10)
        # self.can_create_path = None
        # self.get_create_path_pub = self.node.create_publisher(Bool,'get_create_path',10)

        self.robot_pose_sub = self.node.create_subscription(Float32MultiArray,'/robot_pose',self.robot_pose_callback, 10)
        self.robot_pose = Float32MultiArray()
        self.robot_pose.data = [-2.0,-1.5,-1.8]
        self.do_robot_pose = False

        self.gripper_base_pose_pub = self.node.create_subscription(Float32MultiArray,'gripper_base_pose', self.gripper_base_pose_callback,10)
        self.gripper_base_pose = Float32MultiArray()
        self.gripper_base_pose.data = [0.4,0.1,1.0,1.5,0.0,0.0]
        self.do_gripper_pose = False

        self.follow_trajectory_sub = self.node.create_subscription(String, "/follow_trajectory_status",self.follow_trajectory_callback, 10)
        self.follow_trajectory = "FAILURE"
        self.do_follow_trajectory = False

        self.move_joint_sub = self.node.create_subscription(String, "/move_joint_status",self.move_joint_callback, 10)
        self.move_joint = "FAILURE"
        self.do_move_joint = False

    def initialise(self) -> None:
        """Backup and set a new context.""" 
        self.logger.debug("%s.initialise()[switch context]" % (self.__class__.__name__))
        self.init_time = time.time()

    def find_aruco_callback(self,msg):
        self.logger.info("Do find_aruco callback!!")
        if msg.data == "True":
            self.is_find_marker = True
            self.logger.info("find_aruco")
        else:
            self.is_find_marker = False

    def robot_pose_callback(self,msg):
        self.robot_pose = msg
        self.do_robot_pose = True
    
    def gripper_base_pose_callback(self,msg):
        self.gripper_base_pose = msg
        self.do_gripper_pose = True

    def follow_trajectory_callback(self,msg):
        self.follow_trajectory = msg.data
        self.do_follow_trajectory = True

    def move_joint_callback(self,msg):
        self.move_joint = msg.data
        self.do_move_joint = True

    # def can_create_path_callback(self,msg):
    #     self.logger.info("Do can_create_path callback!!")
    #     self.can_create_path = msg.data
    #     self.get_create_path_pub.publish(Bool(data=True))
    #     print("can create path: ", self.can_create_path)
    #     print("pub get_create_path!!!")
    
    def update(self) -> py_trees.common.Status:
        print("Name: ", self.name)
        if self.blackboard.get(self.name) == True:
            self.behavior_status.data = "Success"
            self.behavior_status_pub.publish(self.behavior_status)
            return py_trees.common.Status.SUCCESS 
        
        # self.logger.info("Do find_aruco loop update!!")
        # self.logger.info("is find marker: "+str(self.is_find_marker))
        # print("can create path: ", self.can_create_path)
        if self.do_robot_pose and self.do_gripper_pose and self.do_follow_trajectory and self.do_move_joint:
            self.logger.info("is find marker: "+str(self.is_find_marker))
            self.logger.info("follow_trajectory: "+str(self.follow_trajectory))
            self.logger.info("move_joint: "+str(self.move_joint))
            self.logger.info("robot_pose: "+str(self.robot_pose.data))
            self.logger.info("gripper_base_pose: "+str(self.gripper_base_pose.data))
            if self.is_find_marker and self.follow_trajectory == 'FAILURE' and self.move_joint == 'FAILURE' : #and \
                # [round(self.robot_pose.data[0],1),round(self.robot_pose.data[1],1),round(self.robot_pose.data[2],1)] == [-2.0,-1.5,-1.8] and \
                #     [round(self.gripper_base_pose.data[0],1),round(self.gripper_base_pose.data[1],1),round(self.gripper_base_pose.data[2],1)] == [0.4,0.1,0.1]:
                has_door_path = String()
                has_door_path.data = "True"
                self.pub_has_door_path.publish(has_door_path)
                self.failure_tick = 0
                self.blackboard.set(self.name, True)
                self.behavior_status.data = "Success"
                self.behavior_status_pub.publish(self.behavior_status)
                self.logger.info("behavior_status SUCCESS: "+str(self.behavior_status))
                return py_trees.common.Status.SUCCESS
            else:
                self.failure_tick += 1
        
        else:
            self.failure_tick += 1
        
        if self.failure_tick >= 30:
            self.logger.info("DO THIS!!!")
            self.do_robot_pose = True
            self.do_gripper_pose = True
            self.do_follow_trajectory = True
            self.do_move_joint = True

        if self.failure_tick >= 50:
            self.behavior_status.data = "FAILURE"
            self.behavior_status_pub.publish(self.behavior_status)
            self.logger.info("behavior_status FAILURE: "+str(self.behavior_status))

        return py_trees.common.Status.FAILURE

    def terminate(self, new_status: py_trees.common.Status) -> None:
        self.logger.info(f"Terminated with status {new_status}")
        self.client = None

# class CreateDoorPath(py_trees.behaviour.Behaviour): ## hold handle task
#     def __init__(self, name, node, id):
#         """Initialise with a behaviour name."""
#         super(CreateDoorPath, self).__init__(name)
#         self.node = node
#         self.name = name+str(id)
#         self.blackboard = py_trees.blackboard.Client(name="Global")
#         self.blackboard.register_key(key=self.name, access=py_trees.common.Access.WRITE)
#         self.blackboard.register_key(key="door_path", access=py_trees.common.Access.WRITE)

#         self.find_aruco_toggle = self.node.create_subscription(String,'is_find_aruco',self.find_aruco_callback, 10) ## topic for check 'find aruco'
#         self.blackboard.set(self.name, False)
#         self.pub_has_door_path = self.node.create_publisher(String, 'has_door_path', 10)
#         self.is_find_marker = False

#         self.failure_tick = 0
#         self.hold_handle_status =String()
#         # self.hold_handle_pub = self.node.create_publisher(String,'/hold_handle_status', 10)
#         self.behavior_status = String()
#         self.behavior_status_pub = self.node.create_publisher(String,'/behavior_status', 10)
#         self.logger.info("Do create door path!!")

#         # self.can_create_path_sub = self.node.create_subscription(Bool,'can_create_path',self.can_create_path_callback, 10)
#         # self.can_create_path = None
#         # self.get_create_path_pub = self.node.create_publisher(Bool,'get_create_path',10)

#         self.robot_pose_sub = self.node.create_subscription(Float32MultiArray,'/robot_pose',self.robot_pose_callback, 10)
#         self.robot_pose = Float32MultiArray()
#         # self.robot_pose.data = [-2.0,-1.5,-1.8]
#         # self.do_robot_pose = False

#         self.gripper_base_pose_pub = self.node.create_subscription(Float32MultiArray,'gripper_base_pose', self.gripper_base_pose_callback,10)
#         self.gripper_base_pose = Float32MultiArray()
#         # self.gripper_base_pose.data = [0.4,0.1,1.0,1.5,0.0,0.0]
#         # self.do_gripper_pose = False

#         # self.follow_trajectory_sub = self.node.create_publisher(String, "/follow_trajectory_status",self.follow_trajectory_callback, 10)
#         # self.follow_trajectory = "FAILURE"
#         # self.do_follow_trajectory = False

#     def initialise(self) -> None:
#         """Backup and set a new context.""" 
#         self.logger.debug("%s.initialise()[switch context]" % (self.__class__.__name__))
#         self.init_time = time.time()

#     def find_aruco_callback(self,msg):
#         self.logger.info("Do find_aruco callback!!")
#         if msg.data == "True":
#             self.is_find_marker = True
#             self.logger.info("find_aruco")
#         else:
#             self.is_find_marker = False

#     def robot_pose_callback(self,msg):
#         self.robot_pose = msg
#         self.do_robot_pose = True
    
#     def gripper_base_pose_callback(self,msg):
#         self.gripper_base_pose = msg
#         self.do_gripper_pose = True

#     def follow_trajectory_callback(self,msg):
#         self.follow_trajectory = msg.data
#         self.do_follow_trajectory = True

#     # def can_create_path_callback(self,msg):
#     #     self.logger.info("Do can_create_path callback!!")
#     #     self.can_create_path = msg.data
#     #     self.get_create_path_pub.publish(Bool(data=True))
#     #     print("can create path: ", self.can_create_path)
#     #     print("pub get_create_path!!!")
    
#     def update(self) -> py_trees.common.Status:
#         print("Name: ", self.name)
#         if self.blackboard.get(self.name) == True:
#             self.behavior_status.data = "Success"
#             self.behavior_status_pub.publish(self.behavior_status)
#             return py_trees.common.Status.SUCCESS 
        
#         # self.logger.info("Do find_aruco loop update!!")
#         # self.logger.info("is find marker: "+str(self.is_find_marker))
#         # print("can create path: ", self.can_create_path)
        
#         if self.is_find_marker: 
#             has_door_path = String()
#             has_door_path.data = "True"
#             self.pub_has_door_path.publish(has_door_path)
#             self.failure_tick = 0
#             self.blackboard.set(self.name, True)
#             self.behavior_status.data = "Success"
#             self.behavior_status_pub.publish(self.behavior_status)
#             self.logger.info("behavior_status SUCCESS: "+str(self.behavior_status))
#             return py_trees.common.Status.SUCCESS
#         else:
#             self.failure_tick += 1

#         if self.failure_tick >= 50:
#             self.behavior_status.data = "FAILURE"
#             self.behavior_status_pub.publish(self.behavior_status)
#             self.logger.info("behavior_status FAILURE: "+str(self.behavior_status))
#         return py_trees.common.Status.FAILURE

#     def terminate(self, new_status: py_trees.common.Status) -> None:
#         self.logger.info(f"Terminated with status {new_status}")
#         self.client = None

class Gripper(py_trees.behaviour.Behaviour): ## 1
    """ Wrapper behavior around the `move_base` action client """

    def __init__(self, name, command ,node):
        super(Gripper, self).__init__(name)
        # self.pose = pose
        self.node = node
        self.command = command
        self.behavior_status = String()
        self.behavior_status_pub = self.node.create_publisher(String,'/behavior_status', 10)

    def initialise(self):
        """ Sends the initial navigation action goal """
        self.gripper_status_pub = self.node.create_publisher(String,'gripper_status',10)

    def update(self):
        """ Checks for the status of the navigation action """
        command = String()
        command.data = self.command
        self.gripper_status_pub.publish(command)
        self.behavior_status.data = "Success"
        self.behavior_status_pub.publish(self.behavior_status)

        return py_trees.common.Status.SUCCESS

    def terminate(self, new_status):
        self.logger.info(f"Terminated with status {new_status}")

class NavigationTo(py_trees.behaviour.Behaviour): ## 1
    """Wrapper behavior around the `move_base` action client"""

    def __init__(self, name, pose, node):
        super(NavigationTo, self).__init__(name)
        self.pose = pose
        self.node = node
        self.blackboard = py_trees.blackboard.Client(name="Global")
        self.name = name

        self.blackboard.register_key(key=self.name, access=py_trees.common.Access.WRITE)
        self.blackboard.set(self.name, False)

        self.door_status_sub = self.node.create_subscription(String,'door_status',self.door_status_callback, 10)
        self.door_status = "close"
        self.do_door_status = False
        self.move_joint_sub = self.node.create_subscription(String, "/move_joint_status",self.move_joint_callback, 10)
        self.move_joint = "FAILURE"
        self.do_move_joint = False

        self.robot_pose_pub = self.node.create_publisher(Float32MultiArray,'robot_pose',10)
        self.robot_pose = Float32MultiArray()
        self.failure_tick = 0

        self.behavior_status = String()
        self.behavior_status_pub = self.node.create_publisher(String,'/behavior_status', 10)
        self.door_traversal_pub = self.node.create_publisher(String, "/door_traversal", 10)

    def initialise(self):
        """Sends the initial navigation action goal"""
        # Check if there is a pose available in the blackboard
        
        if self.blackboard.get(self.name):
            return py_trees.common.Status.SUCCESS 
        self.failure_tick = 0    
        
    def door_status_callback(self,msg):
        self.door_status = msg.data
        self.do_door_status = True

    def move_joint_callback(self,msg):
        self.move_joint = msg.data
        self.do_move_joint = True

    def update(self):
        """Checks for the status of the navigation action"""
        if self.pose == [-2.0, -1.5, -2.1]:
            self.robot_pose.data = self.pose
            self.robot_pose_pub.publish(self.robot_pose)
            self.behavior_status.data = "Success"
            self.behavior_status_pub.publish(self.behavior_status)
            self.logger.info("Do Navigation To InitPose!!")
            return py_trees.common.Status.SUCCESS
        elif self.pose == [-1.0, -8.0, -2.1]:
            if self.do_door_status == True and self.do_move_joint == True and\
                  self.door_status == "open" and self.move_joint == "SUCCESS":
                self.robot_pose.data = self.pose
                self.robot_pose_pub.publish(self.robot_pose)
                self.behavior_status.data = "Success"
                self.behavior_status_pub.publish(self.behavior_status)
                self.door_traversal_pub.publish(String(data="SUCCESS"))
                return py_trees.common.Status.SUCCESS
            else:
                self.failure_tick += 1
                if self.failure_tick >= 10:
                    self.behavior_status.data = "FAILURE"
                    self.behavior_status_pub.publish(self.behavior_status)
                    return py_trees.common.Status.FAILURE
        return py_trees.common.Status.RUNNING

    def terminate(self, new_status):
        self.logger.info(f"Terminated with status {new_status}")

class MobileBaseControl(py_trees.behaviour.Behaviour): ## 1

    def __init__(
            self,
            name: str,
            node,
            topic_name: str="/cmd_vel",
            mode: str="forward",
    ):
        super().__init__(name=name)
        self.topic_name = topic_name
        self.mode = mode
        self.node = node

        self.blackboard = py_trees.blackboard.Client(name="Global")
        self.name = name

        self.blackboard.register_key(key=self.name, access=py_trees.common.Access.WRITE)
        self.blackboard.set(self.name, False)

        self.robot_pose_sub = self.node.create_subscription(Float32MultiArray,'robot_pose',self.robot_pose_callback, 10)
        self.robot_pose_aruco_pub = self.node.create_publisher(Float32MultiArray,'robot_pose_aruco',10)
        self.robot_pose = Float32MultiArray()
        self.robot_pose.data = [0.0,0.0,0.0]
        self.is_start = False
        self.robot_pose_update = None
        self.theta_update = None

        self.sum_delta = 0.0
        self.failure_tick = 0

        self.behavior_status = String()
        self.behavior_status_pub = self.node.create_publisher(String,'/behavior_status', 10)

        self.node.get_logger().info("Initialized")
    
    def robot_pose_callback(self,msg):
        # self.node.get_logger().info("Robot pose callback:"+str(msg.data))
        if self.is_start is False:
            self.robot_pose = msg
            self.theta_update = self.robot_pose.data[2]
            self.is_start = True

    def update(self) -> py_trees.common.Status:
        """
        Annoy the led strip to keep firing every time it ticks over (the led strip will clear itself
        if no command is forthcoming within a certain period of time).
        This behaviour will only finish if it is terminated or priority interrupted from above.

        Returns:
            Always returns :attr:`~py_trees.common.Status.RUNNING`
        """
        self.node.get_logger().info("Updated")
        if self.blackboard.get(self.name):
            return py_trees.common.Status.SUCCESS
        
        self.logger.debug("%s.update()" % self.__class__.__name__)
        # self.node.get_logger().info("Robot pose: "+str(self.robot_pose.data))
        # self.node.get_logger().info("Is start: "+str(self.is_start))
        if self.robot_pose.data[0] != 0.0 and self.robot_pose.data[1] != 0.0:
            if self.is_start:
                if self.mode == "turn_left":
                    print("turn_left")
                    # print("theta_update", self.theta_update)
                    delta_theta = 0.05 #rad
                    delta_max = math.pi*2.0
                    self.sum_delta = self.sum_delta + delta_theta
                    self.theta_update = self.theta_update + delta_theta
                    # self.theta_update = self.theta_update % (2*math.pi)
                    self.robot_pose_aruco_pub.publish(Float32MultiArray(data=[self.robot_pose.data[0],self.robot_pose.data[1],self.theta_update]))

                    if self.sum_delta >= delta_max:
                        self.blackboard.set(self.name, True)
                        self.is_start = False
                        self.behavior_status.data = "Success"
                        self.behavior_status_pub.publish(self.behavior_status)
                        return py_trees.common.Status.SUCCESS
                    
                elif self.mode == "turn_right":
                    delta_theta = -0.05 #rad
                    delta_max = -(math.pi*2.0)
                    self.sum_delta = self.sum_delta + delta_theta
                    self.theta_update = self.theta_update + delta_theta
                    # self.theta_update = self.theta_update % (2*math.pi)
                    self.robot_pose_aruco_pub.publish(Float32MultiArray(data=[self.robot_pose.data[0],self.robot_pose.data[1],self.theta_update]))

                    if self.sum_delta <= delta_max:
                        self.is_start = False
                        return py_trees.common.Status.SUCCESS
        elif self.mode == "stop":
            self.blackboard.set(self.name, True)
            self.behavior_status.data = "Success"
            self.behavior_status_pub.publish(self.behavior_status)
            return py_trees.common.Status.SUCCESS
        else:
            self.failure_tick += 1
            self.node.get_logger().info("Failure tick: "+str(self.failure_tick))
            if self.failure_tick >= 50:
                self.behavior_status.data = "FAILURE"
                self.behavior_status_pub.publish(self.behavior_status)
                self.failure_tick = 0
            return py_trees.common.Status.FAILURE   

        return py_trees.common.Status.RUNNING

    def terminate(self, new_status: py_trees.common.Status):
        """
        Shoot off a clearing command to the led strip.

        Args:
            new_status: the behaviour is transitioning to this new status
        """
        self.logger.debug(
            "{}.terminate({})".format(
                self.qualified_name,
                "{}->{}".format(self.status, new_status) if self.status != new_status else "{}".format(new_status)
            )
        )
        #self.publisher.publish(String(data=""))
        self.logger.info(f"Terminated with status {new_status}")
        self.feedback_message = "cleared"

class MMApproachDoor(py_trees.behaviour.Behaviour): ## 1
    """Wrapper behavior around the `move_base` action client"""
    def __init__(self, name, taskspace ,node):
        super(MMApproachDoor, self).__init__(name)
        # self.pose = pose
        self.client = None
        self.node = node
        self.blackboard = py_trees.blackboard.Blackboard()
        self.blackboard.set(self.name, False)
        self.is_start = False
        
        self.has_door_path_sub = self.node.create_subscription(String,'has_door_path',self.has_door_path_callback, 10)
        self.has_door_path = "None"
        self.robot_pose_pub = self.node.create_publisher(Float32MultiArray,'robot_pose',10)
        
        self.failure_tick = 0
        self.hold_handle_status =String()
        self.hold_handle_pub = self.node.create_publisher(String,'/hold_handle_status', 10)

        self.behavior_status = String()
        self.behavior_status_pub = self.node.create_publisher(String,'/behavior_status', 10)

        self.can_create_path_pub = self.node.create_publisher(Bool,'can_create_path', 10)

    def has_door_path_callback(self,msg):
        # if self.is_start == False:
        self.has_door_path = msg.data
        self.is_start = True

    def update(self):
        """ Checks for the status of the navigation action """
        # If there is a result, we can check the status of the action directly.
        # Otherwise, the action is still running.
        robot_pose = Float32MultiArray()
        robot_pose.data = [-1.9, -2.2, -1.4]
        self.behavior_status.data = "Success"

        if self.blackboard.get(self.name):
            self.robot_pose_pub.publish(robot_pose)
            self.behavior_status_pub.publish(self.behavior_status)
            self.can_create_path_pub.publish(Bool(data=False))
            return py_trees.common.Status.SUCCESS   

        if self.has_door_path != "None":
            if self.has_door_path == "True":
                # robot_pose = Float32MultiArray()
                # robot_pose.data = [-1.9, -2.2, -1.4]
                self.robot_pose_pub.publish(robot_pose)
                # self.behavior_status.data = "Success"
                self.behavior_status_pub.publish(self.behavior_status)
                self.can_create_path_pub.publish(Bool(data=False))
                return py_trees.common.Status.SUCCESS 
            else:
                self.behavior_status.data = "FAILURE"
                self.behavior_status_pub.publish(self.behavior_status)
                return py_trees.common.Status.FAILURE
        else:        
            self.failure_tick += 1
            if self.failure_tick >= 10:
                self.behavior_status.data = "FAILURE"
                self.behavior_status_pub.publish(self.behavior_status)
            return py_trees.common.Status.FAILURE

    def terminate(self, new_status):
        self.logger.info(f"Terminated with status {new_status}")

class MMMoveJoint(py_trees.behaviour.Behaviour): ## 1
    """Wrapper behavior around the `move_base` action client"""

    
    def __init__(self, name, joint ,node):
        super(MMMoveJoint, self).__init__(name)
        # self.pose = pose
        # self.client = None
        self.node = node
        self.blackboard = py_trees.blackboard.Blackboard()
        # self.command = joint
        # self.blackboard = py_trees.blackboard.Client(name="Global")
        self.name = name
        self.blackboard.set(self.name, False)

        # self.is_start = False

        self.behavior_status = String()
        self.behavior_status_pub = self.node.create_publisher(String,'/behavior_status', 10)
        self.hold_handle_status_pub = self.node.create_publisher(String, "/hold_handle_status", 10)
        self.hold_handle_status_sub = self.node.create_subscription(String, "/hold_handle_status",self.hold_handle_callback, 10)
        self.hold_handle_status = "FAILURE"
        self.do_hold_handle = False
        self.gripper_status_sub = self.node.create_subscription(String,'gripper_status',self.gripper_status_callback,10)
        self.gripper_status = ""
        self.do_gripper_status = False
        self.move_joint_status_pub = self.node.create_publisher(String, "/move_joint_status", 10)

        self.failure_tick = 0

    def hold_handle_callback(self,msg):
        self.hold_handle_status = msg.data
        self.do_hold_handle = True

    def gripper_status_callback(self, msg):
        self.gripper_status = msg.data
        self.do_gripper_status = True

    def initialise(self):
        """ Sends the initial navigation action goal """
        if self.blackboard.get(self.name):
            return py_trees.common.Status.SUCCESS

    def update(self):
        if self.blackboard.get(self.name) == True:
            self.behavior_status.data = "Success"
            self.behavior_status_pub.publish(self.behavior_status)
            return py_trees.common.Status.SUCCESS 
        
        if self.do_hold_handle and self.do_gripper_status:
            if self.hold_handle_status == "SUCCESS" and self.gripper_status == "Open":
                self.behavior_status.data = "Success"
                self.behavior_status_pub.publish(self.behavior_status)
                self.move_joint_status_pub.publish(String(data="SUCCESS"))
                self.hold_handle_status_pub.publish(String(data="FAILURE"))
                self.blackboard.set(self.name, True)
                return py_trees.common.Status.SUCCESS
            else:
                self.failure_tick += 1
        
        else:
            self.failure_tick += 1
        
        if self.failure_tick >= 10:
            self.behavior_status.data = "FAILURE"
            self.behavior_status_pub.publish(self.behavior_status)
            self.move_joint_status_pub.publish(String(data="FAILURE"))
            return py_trees.common.Status.FAILURE
        
        return py_trees.common.Status.RUNNING

    def terminate(self, new_status):
        self.logger.info(f"Terminated with status {new_status}")
        self.client = None
        # self.bb.set("target_pose", None)

class MMMoveLinear(py_trees.behaviour.Behaviour): ## 1
    """Wrapper behavior around the `move_base` action client"""

    def __init__(self, name, taskspace ,node):
        super(MMMoveLinear, self).__init__(name)
        self.node = node
        self.blackboard = py_trees.blackboard.Blackboard()
        self.blackboard.set(self.name, False)

        self.has_door_path_sub = self.node.create_subscription(String,'has_door_path',self.has_door_path_callback, 10)
        self.has_door_path = "False"
        self.robot_pose_sub = self.node.create_subscription(Float32MultiArray,'/robot_pose',self.robot_pose_callback, 10)
        self.robot_pose = Float32MultiArray()
        self.robot_pose.data = [0.0,0.0,0.0]
        self.gripper_base_pose_pub = self.node.create_publisher(Float32MultiArray,'gripper_base_pose',10)

        self.failure_tick = 0
        self.hold_handle_status =String()
        self.hold_handle_pub = self.node.create_publisher(String,'/hold_handle_status', 10)
        self.behavior_status = String()
        self.behavior_status_pub = self.node.create_publisher(String,'/behavior_status', 10)
        self.gripper_status_sub = self.node.create_subscription(String,'gripper_status',self.gripper_status_callback, 10)
        self.gripper_status = ""

        self.can_create_path_pub = self.node.create_publisher(Bool,'can_create_path', 10)

    def has_door_path_callback(self,msg):
        self.has_door_path = msg.data

    def robot_pose_callback(self,msg):
        self.robot_pose = msg
    
    def gripper_status_callback(self, msg):
        self.gripper_status = msg.data
            
    def update(self):
        if self.blackboard.get(self.name):
            self.behavior_status.data = "Success"
            self.behavior_status_pub.publish(self.behavior_status)
            self.can_create_path_pub.publish(Bool(data=False))
            return py_trees.common.Status.SUCCESS 

        # print("Is MoveL start:", self.blackboard.get(self.name+str("_is_start")))  
        self.logger.info("Robot pose:"+str(self.robot_pose.data))
        if self.has_door_path == "True":
            if self.robot_pose.data != [0.0,0.0,0.0]:
                if round(self.robot_pose.data[0],1) == -1.9 and round(self.robot_pose.data[1],1) == -2.2 and round(self.robot_pose.data[2],1) == -1.4 and self.gripper_status == "Open":
                    gripper_base_pose = Float32MultiArray()
                    gripper_base_pose.data = [0.8,0.2,1.0,1.5,0.0,0.2]
                    self.gripper_base_pose_pub.publish(gripper_base_pose)
                    self.blackboard.set(self.name, True)
                    self.failure_tick = 0
                    self.behavior_status.data = "Success"
                    self.behavior_status_pub.publish(self.behavior_status)
                    self.can_create_path_pub.publish(Bool(data=False))
                    return py_trees.common.Status.SUCCESS
                else:
                    self.failure_tick += 1
                    if self.failure_tick >= 10:
                        self.behavior_status.data = "FAILURE"
                        self.behavior_status_pub.publish(self.behavior_status)
                    return py_trees.common.Status.FAILURE
            # else:
            #     return py_trees.common.Status.RUNNING
        else:
            # self.response = "FAILURE"
            self.failure_tick += 1
            if self.failure_tick >= 10:
                self.behavior_status.data = "FAILURE"
                self.behavior_status_pub.publish(self.behavior_status)
            return py_trees.common.Status.FAILURE
        
        # return py_trees.common.Status.RUNNING

    def terminate(self, new_status):
        self.logger.info(f"Terminated with status {new_status}")

class MMFollowTrajectory(py_trees.behaviour.Behaviour):
    """Wrapper behavior around the `move_base` action client"""

    
    def __init__(self, name ,node):
        super(MMFollowTrajectory, self).__init__(name)
        self.node = node
        self.name = name
        self.blackboard = py_trees.blackboard.Blackboard()
        self.blackboard.set(self.name, False)

        self.behavior_status = String()
        self.behavior_status_pub = self.node.create_publisher(String,'/behavior_status', 10)

        self.hold_handle_status_pub = self.node.create_publisher(String, "/hold_handle_status", 10)
        self.hold_handle_status_sub = self.node.create_subscription(String, "/hold_handle_status",self.hold_handle_callback, 10)
        self.hold_handle_status = "FAILURE"
        self.do_hold_handle = False

        self.gripper_status_sub = self.node.create_subscription(String,'gripper_status',self.gripper_status_callback,10)
        self.gripper_status = ""
        self.do_gripper_status = False

        self.follow_trajectory_pub = self.node.create_publisher(String, "/follow_trajectory_status", 10)
        self.door_status_pub = self.node.create_publisher(String,'door_status', 10)

        self.failure_tick = 0

    def hold_handle_callback(self,msg):
        self.hold_handle_status = msg.data
        self.do_hold_handle = True

    def gripper_status_callback(self, msg):
        self.gripper_status = msg.data
        self.do_gripper_status = True

    def initialise(self):
        """ Sends the initial navigation action goal """
        pass

    def update(self):
        """ Checks for the status of the navigation action """
        if self.blackboard.get(self.name):
            self.behavior_status.data = "Success"
            self.behavior_status_pub.publish(self.behavior_status)
            self.follow_trajectory_pub.publish(String(data="SUCCESS"))
            self.door_status_pub.publish(String(data="open"))
            return py_trees.common.Status.SUCCESS
        
        self.node.get_logger().info("do_gripper_status:"+str(self.do_gripper_status))
        self.node.get_logger().info("do_hold_handle:"+str(self.do_hold_handle))

        if self.do_hold_handle and self.do_gripper_status:

            self.node.get_logger().info("hold_handle_status:"+str(self.hold_handle_status))
            self.node.get_logger().info("gripper_status:"+str(self.gripper_status))

            if self.hold_handle_status == "SUCCESS" and self.gripper_status == "Closed":
                self.behavior_status.data = "Success"
                self.failure_tick = 0
                self.behavior_status_pub.publish(self.behavior_status)
                self.follow_trajectory_pub.publish(String(data="SUCCESS"))
                self.door_status_pub.publish(String(data="open"))
                self.blackboard.set(self.name, True)
                return py_trees.common.Status.SUCCESS
            else:
                self.failure_tick += 1
        else:
            self.failure_tick += 1

        if self.failure_tick >= 10:
            self.behavior_status.data = "FAILURE"
            self.behavior_status_pub.publish(self.behavior_status)
            self.follow_trajectory_pub.publish(String(data="FAILURE"))
            self.door_status_pub.publish(String(data="close"))
            return py_trees.common.Status.FAILURE
        
        return py_trees.common.Status.RUNNING

    def terminate(self, new_status):
        self.logger.info(f"Terminated with status {new_status}")
        self.client = None
        self.blackboard.set("target_pose", None)
