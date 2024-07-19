#!/usr/bin/env python3

from rclpy.node import Node
from std_srvs.srv import Empty
import numpy as np
import math
from msg_interface.srv import GenBT #, Test
import json
import math
import threading 

from std_msgs.msg import String, Int32, Bool,Float32MultiArray

from rclpy.task import Future
import threading

num_action_max = 7
class RobotController(Node):
    def __init__(self):
        super().__init__('rl_brain')  
        timer_period = 0.1  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)
        self.test_topic = self.create_publisher(String, "/test_topic", 10)
        self.req = Empty.Request()

        self.scores = []
        self.eps_history = []
        self.start = True

        self.gen_bt_client = self.create_client(GenBT, "/gen_bt")
        self.request = GenBT.Request()

        '''Used when train the agent for HoldHandle task'''
        # self.hold_handle_sub = self.create_subscription(String, "/hold_handle_status", self.hold_handle_callback, 10)
        # self.hold_handle_pub = self.create_publisher(String, "/hold_handle_status", 10)
        # self.hold_handle_status = ""

        '''Used when train the agent for DoorTraversal task'''
        self.door_status = ""
        self.door_traversal_status_sub = self.create_subscription(String, "/door_traversal_status", self.door_traversal_callback, 10)
        self.door_traversal_status_pub = self.create_publisher(String, "/door_traversal_status", 10)
        self.door_traversal = ""

        self.num_action = Int32()
        self.num_action.data = 1
        self.tree_memory = []
        self.collection_position = [1]
        self.subtree_num = [0]

        self.behavior_data = String()
        self.behavior_status_pub = self.create_publisher(String,'/behavior_status', 10)
        self.ready_sub = self.create_subscription(Bool,'/ready', self.ready_callback, 10)

        self.ready = Bool()
        self.gen_toggle = False

        self._lock = threading.Lock()
        self.future = []
        self.result = 1

        self.find_aruco_pub = self.create_publisher(String,'is_find_aruco',10)
        self.has_door_path_pub = self.create_publisher(String,'has_door_path',10)
        self.gripper_status_pub = self.create_publisher(String,'gripper_status',10)
        self.robot_pose_pub = self.create_publisher(Float32MultiArray,'robot_pose',10)
        self.gripper_base_pose_pub = self.create_publisher(Float32MultiArray,'gripper_base_pose',10)
        self.find_aruco = "False"
        self.has_door_path = "False"
        self.gripper_status = "Open"
        self.robot_pose = []
        self.gripper_base_pose = []
        self.BT_string_state = []

        self.behavior_status = "FAILURE"
        self.BT_status = "FAILURE"
        self.step_num = 0
        self.d_old = 0

        self.chosen_action_pub = self.create_publisher(Int32,'/chosen_action', 10)

        self.iteration = 0
        self.iteration_pub = self.create_publisher(Int32, "/iteration", 10)
        self.can_create_path_pub = self.create_publisher(Bool,'can_create_path', 10)
#-------------------------BT-------------------------#
    class BackwardChainedFormat(Node):
        def __init__(self, condition, name, IsPrimitive, IsStart):
            super().__init__("backward_chained_format")
            self.get_logger().info("Init controller")
            self.name = name
            self.post_condition = condition
            self.is_primitive = IsPrimitive
            self.sub_tree = []
            self.start_tree = IsStart
            self.sub_tree.append('s(')

    def create_subtree(self,action,is_condition,post_condition, idx_action_position):
        '''set post_condition command and initial string of BT'''
        command = self.command_bt(post_condition)
        subtree_node = []
        if self.start == True: 
            self.tree_memory.append('s(')

        self.start = False   
        sub_action = action

        '''get last position of the choosen layer of the BT'''
        print("collection_position:",self.collection_position)
        action_position = self.collection_position[idx_action_position]

        '''insert sub_action to the tree_memory at the choosen layer of the BT'''
        if action_position == len(self.tree_memory):
            for i in range(len(sub_action)):
                self.tree_memory.append(sub_action[i])
        else:
            for i in range(len(sub_action)):
                self.tree_memory.insert(action_position+i-1,sub_action[i])

        '''update collection_position variable'''
        for j in range(0,len(self.collection_position)):
            if self.collection_position[idx_action_position] <= self.collection_position[j]:
                self.collection_position[j] += len(sub_action)

        '''if the chosen action is condition behavior (BT has new layer): add new number of position to the collection_position variable'''
        if is_condition == True:
            new_position = self.collection_position[idx_action_position]- 1
            self.collection_position.append(new_position)
            self.subtree_num.append(0)
        
        '''create list of BT string'''
        for k in range(len(self.tree_memory)):
            subtree_node.append(self.tree_memory[k])
        if subtree_node[-1] == 's(':
            subtree_node.pop(1)
            self.tree_memory.pop(1)
        
        subtree_node.append(command)
        subtree_node.append(')')
        json_format = {"key":subtree_node}
        return json_format
    
    def send_request(self, req):
        '''send BT in json format and number of action requests to AutonomyBehavior Node'''
        self.request.generate_bt = req
        self.request.action_num = self.num_action
        self.gen_bt_client.call_async(self.request)
        print("Send gen_bt request successfully!!!")
        return
            
    def command_bt(self, command):
        # if command == "isGripperOpen":
        #     command_bt = {"command":"isGripper", "name":"0", "state":"Open"}

        # elif command == "OpenGripper":
        #     command_bt = {"command":"OpenGripper"}
        
        # elif command == "CreateDoorPath":
        #     command_bt = {"command":"CreateDoorPath", "id":582}
        
        # elif command == "MMMoveLinear":
        #     command_bt = {"command":"MMMoveLinear","name":"1"}
        
        # elif command == "isGripperClose":
        #     command_bt = {"command":"isGripper", "name":"3", "state":"Closed"}
        
        # elif command == "CloseGripper":
        #     command_bt = {"command":"ClosedGripper"}
        
        # elif command == "MMFollowTrajectory":
        #     command_bt = {"command":"MMFollowTrajectory", "condition":1}

        if command == "isHoldHandle":
            command_bt = {"command":"isHoldHandle", "name":"0"}

        elif command == "isCreateDoorPath":
            command_bt = {"command":"isCreateDoorPath", "name":"0"}

        elif command == "isDoorTraversal":
            command_bt = {"command":"isDoorTraversal", "name":"0"}
        return command_bt

    def gen_bt(self,action,is_condition,post_condition,idx_action_position):
        '''generate BT string and send request to execute BT'''
        req = String()
        sub_tree_json = self.create_subtree(action,is_condition,post_condition,idx_action_position)
        req.data = json.dumps(sub_tree_json)
        response_gen = self.send_request(req)
        
    def cal_distance(self, parent_pose, child_pose):
        distance = np.abs(math.sqrt((parent_pose[0] - child_pose[0])**2 + (parent_pose[1] - child_pose[1])**2 + (parent_pose[2] - child_pose[2])**2))
        return distance
    
    def timer_callback(self):
        action_num_old = self.num_action.data
        if self.behavior_data.data == "FAILURE" or self.door_traversal == "SUCCESS" or (self.num_action.data == num_action_max and self.door_traversal == "FAILURE"):
            print("!! Regenerate BT !!")
            self.behavior_status = self.behavior_data.data
            self.gen_toggle = False
            self.step_num = self.num_action.data
            self.num_action.data = 1
            self.start = True
            # self.tree_memory = []
            self.behavior_data.data = "INIT"
            self.behavior_status_pub.publish(self.behavior_data)
            reset_status = String()
            reset_status.data = "INIT"
            self.door_traversal_status_pub.publish(reset_status)
            self.future.set_result(self.result)

        elif self.door_traversal == "FAILURE" and self.num_action.data < num_action_max and self.gen_toggle == True:
            print("!! Add BT !!")
            # self.get_logger().info("self.ready.data:"+str(self.ready.data))
            if self.ready.data == True:
                self.BT_status = self.door_traversal
                reset_status = String()
                reset_status.data = "INIT"
                self.door_traversal_status_pub.publish(reset_status)
                self.step_num = self.num_action.data
                self.num_action.data += 1
                # self.gen_bt("isHoldHandle")
                self.future.set_result(self.result)
        else:
            pass
            
        if action_num_old != self.num_action.data:
            print("num_action:",self.num_action.data)

    def ready_callback(self, msg):
        self.ready = msg

    '''Used when train the agent for HoldHandle task'''
    # def hold_handle_callback(self, msg):
    #     self.hold_handle_status = msg.data

    '''Used when train the agent for DoorTraversal task'''
    def door_traversal_callback(self, msg):
        self.door_traversal = msg.data
    
    def reset_topic(self):
        self.find_aruco = "False"
        self.has_door_path = "False"
        self.gripper_status = "Open"
        self.robot_pose = [-2.0, -1.5, -1.8] #[-2.0, -1.5, -2.1]
        self.gripper_base_pose = [0.4,0.1,1.0,1.5,0.0,0.0]

        iteration = Int32()
        self.iteration += 1
        iteration.data = self.iteration
        self.iteration_pub.publish(iteration)
        print("iteration:",self.iteration)

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

        self.BT_string_state = []
        self.tree_memory = []
        self.collection_position = [1]
        self.subtree_num = [0]
        self.d_old = 0.0
        print("Reset topic successfully!!!")

class Callback(Node):
    def __init__(self):
        super().__init__("callback_node")
        self.timer = self.create_timer(0.1,self.timer_callback)
        # self.robot_pose_sub = self.create_subscription(Float32MultiArray,'robot_pose',self.robot_pose_callback,10)
        self.behavior_data = String()
        self.behavior_status_sub = self.create_subscription(String,'/behavior_status', self.behavior_status_callback, 10)
        self.hold_handle_sub = self.create_subscription(String, "/hold_handle_status", self.hold_handle_callback, 10)
        self.hold_handle_pub = self.create_publisher(String, "/hold_handle_status", 10)
        self.hold_handle_status = "FAILURE"
        self.door_traversal_status_sub = self.create_subscription(String, "/door_traversal_status", self.door_traversal_callback, 10)
        self.door_traversal_status_pub = self.create_publisher(String, "/door_traversal_status", 10)
        self.door_traversal = ""

        self.do_behavior_callback = False
        self.do_BT_callback = False
        self.future = []
        self._lock = threading.Lock()
        self.result = []

        self.pub_toggle_pub = self.create_publisher(Bool, "/pub_toggle", 10)

        # self.find_aruco_sub = self.create_subscription(String,'is_find_aruco',self.find_aruco_callback, 10)
        # self.has_door_path_sub = self.create_subscription(String,'has_door_path',self.has_door_path_callback,10)
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
        self.do_door_status = False

        self.do_aruco_callback = False
        self.do_door_path_callback = False
        self.do_gripper_status_callback = False
        self.do_robot_pose_callback = False
        self.do_gripper_pose_callback = False
        self.do_hold_handle_callback = False
        self.failure_tick = 0

    def timer_callback(self):
        self.failure_tick += 1
        print(self.do_behavior_callback, self.do_BT_callback, self.do_hold_handle_callback, self.do_door_status, self.do_gripper_status_callback, self.do_robot_pose_callback, self.do_gripper_pose_callback)
        if (self.do_behavior_callback is True and self.do_BT_callback is True and self.do_hold_handle_callback is True \
            and self.do_door_status is True and self.do_gripper_status_callback is True and self.do_robot_pose_callback is True and self.do_gripper_pose_callback is True and self.do_hold_handle_callback is True)\
                or (self.failure_tick > 20):
            if self.do_BT_callback is False:
                self.door_traversal_status_pub.publish(String(data="FAILURE"))
            self.failure_tick = 0
            self.result.append(self.behavior_data.data)
            self.result.append(self.door_traversal)
            # self.result.append(self.find_aruco)
            # self.result.append(self.has_door_path)
            self.result.append(self.hold_handle_status)
            self.result.append(self.door_status)

            self.result.append(self.gripper_status)
            self.result.append(self.robot_pose)
            self.result.append(self.gripper_base_pose)
            
            self.pub_toggle_pub.publish(Bool(data=True))
            self.get_logger().info("Publish toggle_pub: True")
            self.future.set_result(self.result)
        self.pub_toggle_pub.publish(Bool(data=False))
        self.get_logger().info("Publish toggle_pub: False")

    def behavior_status_callback(self, msg):
        self.behavior_data = msg
        self.do_behavior_callback = True
        # self.get_logger().info("Callback behavior_status:"+str(msg.data))

    def door_traversal_callback(self, msg):
        self.door_traversal = msg.data
        self.do_BT_callback = True
        print("door_traversal:",msg.data)
        # self.get_logger().info("Callbackhold_handle_status:"+str(msg.data))

    # def find_aruco_callback(self, msg):
    #     print("find_aruco callback:",msg.data)
    #     self.do_aruco_callback = True
    #     self.find_aruco = msg.data
    
    # def has_door_path_callback(self, msg):
    #     print("has_door_path callback:",msg.data)
    #     self.do_door_path_callback = True
    #     self.has_door_path = msg.data

    def gripper_status_callback(self, msg):
        print("gripper_status callback:",msg.data)
        self.do_gripper_status_callback = True
        self.gripper_status = msg.data

    def robot_pose_callback(self, msg):
        print("robot_pose callback:",msg.data)
        self.do_robot_pose_callback = True
        self.robot_pose = msg.data

    def gripper_base_pose_callback(self, msg):
        self.do_gripper_pose_callback = True
        print("gripper_base_pose callback:",msg.data)
        self.gripper_base_pose = msg.data

    def hold_handle_callback(self, msg):
        self.hold_handle_status = msg.data
        self.do_hold_handle_callback = True
        print("hold_handle_status:",msg.data)

    def door_status_callback(self,msg):
        self.door_status = msg.data
        self.do_door_status = True
        print("door_status:",msg.data)

    def spin_future(self) -> Future:
        with self._lock:
            future = Future()
            self.future = future
        return future