import rclpy
import numpy as np
from robot_controller import RobotController, Callback
import math
from rclpy.task import Future
from rclpy.task import Future

num_action_max = 5

class HoldHandleEnv(RobotController):
    """
    This class inherits from RobotController Node and contain nine functions as follows:
        - __init__(): initialize some global variables
        - convert_action(): convert action number to list of strings that represent each BT action
        - step(): take action and return all of information that occur after the agent acts in each timestep.
        - spin(): spin the RobotController node
        - spin_future(): Do spin_until_future_complete
        - get_obs(): get dictionary of observation from the environment
        - get_information(): get dictionary of information from the environment
        - cal_reward(): calculate reward from the earned information
        - reset(): reset the variables, topics, and get the initial observation and information
    """
    def __init__(self):
        super().__init__()
        self.get_logger().info("OpenDoorEnv: initializing")
        self.num_step = 0
        self.reset_topic()

        target_pose = [-1.0,-8.0,-2.1]
        robot_pose = [-2.0, -1.5, -1.8]
        dist_goal_robot = self.cal_distance(target_pose, robot_pose)
        self.d_old = dist_goal_robot
    
    def convert_action(self, action_num):
        is_condition = False
        create_door_path = [{"command":"CreateDoorPath", "id":238}]
        approach_door = [{"command":"MMApproachDoor", "id":238}]
        move_linear = [{"command":"MMMoveLinear","name":"0"}]
        open_gripper = [{"command":"OpenGripper"}]
        close_gripper = [{"command":"ClosedGripper"}]

        is_gripper_closed = [
                                'f(',
                                    {"command":"isGripper", "name":"1", "state":"Closed"},
                                    's(',')',
                                ')'
                            ]
        is_gripper_open = [
                                'f(',
                                    {"command":"isGripper", "name":"1", "state":"Open"},
                                    's(',')',
                                ')'
                            ]

        if action_num == 0:
            action = create_door_path
        elif action_num == 1:
            action = approach_door
        elif action_num == 2:
            action = move_linear
        elif action_num == 3:
            action = close_gripper
        elif action_num == 4:
            action = open_gripper
        elif action_num == 5:
            action = is_gripper_closed
            is_condition = True
        elif action_num == 6:
            action = is_gripper_open
            is_condition = True

        return action, is_condition
    
    def step(self,action_num,idx_action_position,returns):
        self.get_logger().info("OpenDoorEnv: step")

        '''set variables'''
        action = 0
        tree_condition = "isHoldHandle"
        self.do_step = True
        self.num_step += 1
        action_num = int(action_num)
        idx_action_position = int(idx_action_position)

        self.get_logger().info("action_num: "+str(action_num))

        '''Convert action, send service to execute the BT, and spin the RobotController Node'''
        action,is_condition = self.convert_action(action_num)
        self.gen_bt(action,is_condition,tree_condition,idx_action_position)
        self.spin()

        '''Spin the Callback Node to get the information from the environment after the BT is terminated'''
        callback_node = Callback()
        future = callback_node.spin_future()
        rclpy.spin_until_future_complete(callback_node,future)
        self.behavior_data.data = future.result()[0]
        self.BT_status = future.result()[1]
        self.find_aruco = future.result()[2]
        self.has_door_path = future.result()[3]
        self.gripper_status = future.result()[4]
        self.robot_pose = future.result()[5]
        self.gripper_base_pose = future.result()[6]
        callback_node.destroy_node()
        
        '''Get observation, information, and calculate reward'''
        observation = self.get_obs(self.find_aruco,self.has_door_path,self.gripper_status,self.robot_pose,self.gripper_base_pose)
        info = self._get_information()
        reward,returns = self.cal_reward(info,returns)
        subtree_num = self.subtree_num

        self.get_logger().info("find aruco: "+str(self.find_aruco))
        self.get_logger().info("observation: "+str(observation))

        '''Check if the episode is done or not'''
        if self.behavior_data.data == "FAILURE" or self.BT_status == "SUCCESS" or self.num_action.data == num_action_max: #or self.subtree_num[idx_action_position] >= 6 #(self.num_action.data == num_action_max and self.hold_handle_status == "FAILURE")
            done = True
        else:
            done = False
        
        return observation, reward, done, info,subtree_num,self.num_action.data,returns
    
    def spin(self):
        self.get_logger().info("Start spin env!!")
        '''
            Do spin_until_future_complete: when do step function
            Do spin_once: when do reset function 
        '''
        if self.do_step:
            self.gen_toggle = True
            future = self.spin_future()
            rclpy.spin_until_future_complete(self,future)
        else:
            rclpy.spin_once(self)

        self.get_logger().info("Spin env success!!")

    def spin_future(self) -> Future:
        print("Do spin_future!!")

        with self._lock:
            future = Future()
            self.future = future
        return future

    def get_obs(self,find_aruco,has_door_path,gripper_status,robot_pose,gripper_base_pose):
        self.get_logger().info("OpenDoorEnv: get_obs")

        '''Convert string of observation to integer or float number'''
        gripper_status_ = -1
        find_aruco_status = 0
        have_door_path = -1

        if find_aruco == "True":
            find_aruco_status = 1
        elif self.find_aruco == "False":
            find_aruco_status = 0

        if has_door_path == "True":
            have_door_path = 1
        elif has_door_path == "False":
            have_door_path = 0

        if gripper_status == "Open":
            gripper_status_ = 1
        elif gripper_status == "Closed":
            gripper_status_ = 0

        observation =   {
                            "find_aruco_status":np.array([find_aruco_status], dtype=np.int16),
                            "have_door_path":np.array([have_door_path], dtype=np.int16),
                            "gripper_status":np.array([gripper_status_], dtype=np.int16),
                            "robot_pose":np.array(robot_pose, dtype=np.float32),
                            "gripper_base_pose":np.array(gripper_base_pose, dtype=np.float32),
                            "BT_string": str(self.BT_string_state)
                        }

        self.BT_string_state = self.tree_memory
        return observation
    
    def _get_information(self): 
        self.get_logger().info("OpenDoorEnv: get_information")

        '''set varibles'''
        behavior_status = 0
        BT_status = 0
        find_aruco = 0

        '''Convert string of information to integer number'''
        if self.behavior_data.data == "Success":
            behavior_status = 0
        elif self.behavior_data.data == "FAILURE":
            behavior_status = 1

        if self.BT_status == "SUCCESS":
            BT_status = 1
        elif self.BT_status == "FAILURE":  
            BT_status = 0

        if self.find_aruco == "True":
            find_aruco = 1

        '''Calculate distance between end-effector and goal'''
        target_pose = [-2.7,-2.4,-1.0]
        gripper_base_pose = [0,0,0]
        gripper_base_pose[0] = self.robot_pose[0]-self.gripper_base_pose[0]
        gripper_base_pose[1] = self.robot_pose[1]-self.gripper_base_pose[1]
        gripper_base_pose[2] = self.robot_pose[2]
        dist_handle_gripper = self.cal_distance(target_pose, gripper_base_pose)

        info = {
                    "behavior_status": behavior_status,
                    "BT_status": BT_status,
                    "distance_handle_gripper": dist_handle_gripper,
                    "find_aruco": find_aruco
                }
        
        self.get_logger().info("Get info behavior_data"+str(self.behavior_data))
        self.get_logger().info("Get info BT_status"+str(self.BT_status))
        self.get_logger().info("Get info robot_pose"+str(self.robot_pose))
        self.get_logger().info("Get info gripper_base_pose"+str(self.gripper_base_pose))
        self.get_logger().info("OpenDoorEnv: _get_information success!!")
        return info
    
    def cal_reward(self, info,returns):
        self.get_logger().info("OpenDoorEnv: cal_reward")

        '''set variables'''
        behavior_status = info["behavior_status"]
        BT_status = info["BT_status"]
        distance_handle_gripper = info["distance_handle_gripper"]
        d_max = 1.2
        d_min = 0.4

        '''Check delta_distance'''
        D = 0.0
        if (round(self.d_old,3)-round(distance_handle_gripper,3)) == 0.0:
            D = 1.0
        
        '''set weights for each reward component'''
        w_d = 20.0
        w_pd = -20.0
        w_s = 100.0
        w_cb = -20.0
        w_n = 10.0

        '''Calculate reward'''
        reward = 0
        reward_d = w_d*((distance_handle_gripper-d_max)/(d_min-d_max))
        reward_pd = w_pd*D
        reward_s = w_s*BT_status
        reward_cb = w_cb*behavior_status
        reward_n = (BT_status*(w_n*math.exp(-((self.step_num-1)/(num_action_max-1))/0.5)))

        reward = reward_d+reward_pd+reward_s+reward_cb+reward_n
        self.get_logger().info("reward:"+str(reward))

        '''Update returns'''
        returns[0]+=reward_d
        returns[1]+=reward_pd
        returns[2]+=reward_s
        returns[3]+=reward_cb
        returns[4]+=reward_n
        returns[5]+=reward

        '''Update old distance variable'''
        self.d_old = distance_handle_gripper
        
        return reward, returns
        
    def reset(self, seed=None, options=None):
        self.get_logger().info("OpenDoorEnv: reset")

        '''spin and reset topics'''
        self.do_step = False
        self.spin()
        self.reset_topic()

        '''set initial observation and information, and return dictionary of them'''
        find_aruco = "False"
        has_door_path = "False"
        gripper_status = "Open"
        robot_pose = [-2.0, -1.5, -1.8]
        gripper_base_pose = [0.4,0.1,1.0,1.5,0.0,0.0]
        observation = self.get_obs(find_aruco,has_door_path,gripper_status,robot_pose,gripper_base_pose)
        info = self._get_information()

        return observation, info