#!/usr/bin/env python3

import rclpy
from gymnasium import Env
from gymnasium.spaces import Dict, Box, Text
import numpy as np
# from hospital_robot_spawner.robot_controller import RobotController, Callback
from robot_controller import RobotController, Callback


import math
#from rcl_interfaces.srv import GetParameters
import json
from std_msgs.msg import Bool
from rclpy.task import Future

from rclpy.node import Node
from tf2_ros import TransformException
from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener
from rclpy.task import Future
import threading
import torch
from std_msgs.msg import Int32


num_action_max = 7
# distance_max = 2.0

class HospitalBotEnv(RobotController, Env):
    """
    This class defines the RL environment. Here are defined:
        - Action space
        - State space
        - Target location
    
    The methods available are:
        - step: makes a step of the current episode
        - reset: resets the simulation environment when an episode is finished
        - close: terminates the environment

    This class inherits from both RobotController and Env.
    Env is a standard class of Gymnasium library which defines the basic needs of an RL environment.
    
    RobotController is a ROS2 Node used to control the agent. It includes the following attributes:
        - _agent_location: current position of the robot (in gazebo's coordinate system)
        - _laser_reads: current scan of the LIDAR
    
    And the following methods (only the ones usefull here):
        - send_velocity_command: imposes a velocity to the agent (the robot)
        - call_reset_simulation_service: resets the simulation
        - call_reset_robot_service: resets the robot position to desired position
        - call_reset_target_service: resets the target position to desired position
    """
    
    def __init__(self):
        super().__init__()
        self.get_logger().info("OpenDoorEnv: initializing")
        #initialize variables
        self.num_step = 0
        self.action_space = Box(low=np.array([0]), high=np.array([6]), dtype=np.int16)
        self.observation_space = Dict(
                                        {
                                            "find_aruco_status": Box(low=np.array([0]), high=np.array([1]), dtype=np.int16),
                                            "have_door_path": Box(low=np.array([0]), high=np.array([1]), dtype=np.int16),
                                            "gripper_status": Box(low=np.array([0]), high=np.array([1]), dtype=np.int16),
                                            "robot_pose" : Box(low=np.array([-10.0,-10.0,-3.5]), high=np.array([5.0,5.0,3.5]), dtype=np.float32),
                                            "gripper_base_pose" : Box(low=np.array([0.0,0.0,0.0,0.0,0.0,0.0]), high=np.array([2.0,2.0,2.0,2.0,2.0,2.0]), dtype=np.float32), ## do not fixed
                                            "BT_string": Text(max_length=50000, min_length = 0)
                                        }
                                    )
        self.reset_topic()
    
    def convert_action(self, action_num,iteration):
        is_condition = False

        # create_door_path = [{"command":"CreateDoorPath", "id":238}]
        # approach_door = [{"command":"MMApproachDoor", "id":238}]
        # move_linear = [{"command":"MMMoveLinear","name":"0"}]
        open_gripper = [{"command":"OpenGripper"}] # not sure
        close_gripper = [{"command":"ClosedGripper"}]

        is_gripper_closed = [
                                'f(',
                                    {"command":"isGripper", "name":str(iteration), "state":"Closed"},
                                    's(',')',
                                ')'
                            ]
        is_gripper_open = [
                                'f(',
                                    {"command":"isGripper", "name":str(iteration), "state":"Open"},
                                    's(',')',
                                ')'
                            ]

        hold_handle =   ['f(',
                                {"command":"isHoldHandle", "name":str(iteration)},
                                's(',
                                    {"command":"CreateDoorPath","name":str(iteration), "id":238}, 
                                    {"command":"MMApproachDoor", "id":238}, 
                                    {"command":"MMMoveLinear","name":str(iteration)}, 
                                    {"command":"ClosedGripper"},
                                ')',
                            ')'
                        ]
        follow_trajectory = [{"command":"MMFollowTrajectory", "name":str(iteration)}]
        navigate_to_goal = [{"command":"NavigateTo", "name":"Goal"+str(iteration) ,"pose":[-1.0, -8.0, -2.1]}]
        move_joint = [{"command":"MMMoveJoint", "name":str(iteration), "joint":[0.0,-1.6,2.3,-0.75,1.57,-1.57]}]

        if action_num == 0:
            action = hold_handle
        elif action_num == 1:
            action = follow_trajectory
        elif action_num == 2:
            action = navigate_to_goal
        elif action_num == 3:
            action = close_gripper
        elif action_num == 4:
            action = open_gripper
        # elif action_num == 5:
        #     action = is_gripper_closed
        #     is_condition = True
        # elif action_num == 6:
        #     action = is_gripper_open
        #     is_condition = True
        elif action_num == 5:
            action = move_joint
        return action, is_condition
    
    def step(self,action_num,idx_action_position,returns,iteration):
        '''
        parameter action: string list of whole BT from learning 
        '''
        action = 0
        action_num = int(action_num)
        idx_action_position = int(idx_action_position)
        self.get_logger().info("action_num: "+str(action_num))
        action,is_condition = self.convert_action(action_num,iteration)
        self.get_logger().info("OpenDoorEnv: step")
        self.do_step = True
        self.num_step += 1
        tree_condition = "isDoorTraversal" # isCreateDoorPath
        self.gen_bt(action,is_condition,tree_condition,idx_action_position)
        self.spin()

        callback_node = Callback()
        future = callback_node.spin_future()
        rclpy.spin_until_future_complete(callback_node,future)
        self.behavior_data.data = future.result()[0]
        self.BT_status = future.result()[1]
        self.hold_handle_status = future.result()[2]
        self.door_status = future.result()[3]
        self.gripper_status = future.result()[4]
        self.robot_pose = future.result()[5]
        self.gripper_base_pose = future.result()[6]
        callback_node.destroy_node()

        observation = self.get_obs(self.hold_handle_status,self.door_status,self.gripper_status,self.robot_pose,self.gripper_base_pose)
        
        # self.get_logger().info("find aruco: "+str(self.find_aruco))
        self.get_logger().info("observation: "+str(observation))

        info = self._get_information()
        reward,returns = self.cal_reward(info,returns)
        subtree_num = self.subtree_num

        if self.behavior_data.data == "FAILURE" or self.BT_status == "SUCCESS" or self.num_action.data == num_action_max: #or self.subtree_num[idx_action_position] >= 6 #(self.num_action.data == num_action_max and self.hold_handle_status == "FAILURE")
            done = True
        else:
            done = False
        
        return observation, reward, done, info,subtree_num,self.num_action.data,returns
    
    def spin(self):
        # spin the BT node -> call gen_bt request to GenBehaviorTree Node
        self.get_logger().info("Start spin env!!")
        if self.do_step:
            self.gen_toggle = True
            future = self.spin_future()
            rclpy.spin_until_future_complete(self,future)
        else:
            rclpy.spin_once(self)

        self.get_logger().info("Spin env success!!")

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

    def get_obs(self,hold_handle_status,door_status,gripper_status,robot_pose,gripper_base_pose):
        self.get_logger().info("OpenDoorEnv: get_obs")
        gripper_status_ = -1
        find_aruco_status = 0
        have_door_path = -1
        hold_handle_status_ = 0
        door_status_ = 0

        if hold_handle_status == "SUCCESS":
            hold_handle_status_ = 1
        elif hold_handle_status == "FAILURE":
            hold_handle_status_ = 0

        if door_status == "open":
            door_status_ = 1
        elif door_status == "close":
            door_status_ = 0

        if gripper_status == "Open":
            gripper_status_ = 1
        elif gripper_status == "Closed":
            gripper_status_ = 0

        observation =   {
                            "hold_handle_status":np.array([hold_handle_status_], dtype=np.int16),
                            "door_status":np.array([door_status_], dtype=np.int16),
                            "gripper_status":np.array([gripper_status_], dtype=np.int16),
                            "robot_pose":np.array(robot_pose, dtype=np.float32),
                            "gripper_base_pose":np.array(gripper_base_pose, dtype=np.float32),
                            "BT_string": str(self.BT_string_state)
                        }
        # print("observation:", observation)
        self.BT_string_state = self.tree_memory
        return observation
    
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
    
    def cal_reward(self, info,returns):
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
        reward_d = w_d*(((dist_goal_robot-d_max)/(d_min-d_max))+door_status)
        # print("reward_d:",reward_d,", type:",type(reward_d))
        reward_pd = w_pd*D
        reward_s = w_s*BT_status
        reward_cb = w_cb*behavior_status
        reward_n = (BT_status*(w_n*math.exp(-((self.step_num-1)/(num_action_max-1))/0.5)))

        reward = reward_d+reward_pd+reward_s+reward_cb+reward_n

        returns[0]+=reward_d
        returns[1]+=reward_pd
        returns[2]+=reward_s
        returns[3]+=reward_cb
        returns[4]+=reward_n
        returns[5]+=reward

        '''------------------------------------------------------------'''
        
        # reward = -(1.5*behavior_status)+(10.0*BT_status)-(5.0*distance_handle_gripper)-1.0 #+(find_aruco*5.0)
        # if reward < 0.0:
        #     reward=reward*(self.step_num/2)
        # else:
        #     reward=reward/(self.step_num/2)

        # self.get_logger().info("behavior status:"+str(behavior_status))
        self.d_old = dist_goal_robot
        self.get_logger().info("reward:"+str(reward))
        return reward, returns
        
    def reset(self, seed=None, options=None):

        self.get_logger().info("OpenDoorEnv: reset")
        self.do_step = False
        self.spin()
        self.reset_topic()
        find_aruco = "False"
        has_door_path = "False"
        gripper_status = "Open"
        robot_pose = [-2.0, -1.5, -1.8] #[-2.0, -1.5, -2.1]
        gripper_base_pose = [0.4,0.1,1.0,1.5,0.0,0.0]
        observation = self.get_obs(find_aruco,has_door_path,gripper_status,robot_pose,gripper_base_pose)
        info = self._get_information()
        # self.get_logger().info("-------------------------------------------")
        return observation, info
    
    def close(self):
        self.destroy_node()