#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
# from hospital_robot_spawner.MonteCarloGLIE import MonteCarloControlGLIE
# from hospital_robot_spawner.nStepTD import nStepTD
# from hospital_robot_spawner.Q_learning import QLearning
# from hospital_robot_spawner.hospitalbot_env import HospitalBotEnv

from MonteCarloGLIE import MonteCarloControlGLIE
from nStepTD import nStepTD
from Q_learning import QLearning
from hospitalbot_env import HospitalBotEnv

import pickle
import json
import torch
class TrainingNode(Node):

    def __init__(self):
        super().__init__("hospitalbot_training", allow_undeclared_parameters=True, automatically_declare_parameters_from_overrides=True)
        self._training_mode = "training"

def main(args=None):

    # Initialize the training node to get the desired parameters
    rclpy.init()
    node = TrainingNode()
    node.get_logger().info("Training node has been created")
    node.get_logger().info("Training mode: " + str(node._training_mode))

    env = HospitalBotEnv()
    if node._training_mode == "training":
        f = 'test.pt'
        # policy,returns,BT_string_dict,reward_report,action_num_report,trajectory_memory = QLearning(env=env,seed=1, num_episodes=1000, model = None, epsilon=1.0,filename=f)
        episode = 6000
        print("Training for ", episode, " episodes")
        print("Saving the model to ", f)
        print("----------------------Training the model----------------------")
        policy,returns,BT_string_dict,reward_report,action_num_report,trajectory_memory,return_memory = MonteCarloControlGLIE(env=env,seed=0, num_episodes=episode, model = None, epsilon=1.0,filename=f)
        # model = dict()
        # model["policy"] = policy
        # model["returns"] = dict(returns)
        # model["BT_string_dict"] = BT_string_dict
        # model["reward_report"] = reward_report
        # model["action_num_report"] = action_num_report
        # model["trajectory_memory"] = trajectory_memory
        # torch.save(model, f)

    elif node._training_mode == "retraining":
        f = 'EX_DoorTraversal_MC_6000_retrain15.pt'
        model = torch.load('EX_DoorTraversal_MC_6000_retrain14.pt')

        episode = 145
        print("Training for ", episode, " episodes")
        print("Saving the model to ", f)
        print("----------------------Training the model----------------------")
        policy,returns,BT_string_dict,reward_report,action_num_report,trajectory_memory,return_memory = MonteCarloControlGLIE(env=env,seed=0, num_episodes=episode, model=model,epsilon=1.0,filename=f)
        # model["policy"] = policy
        # model["returns"] = dict(returns)
        # model["BT_string_dict"] = BT_string_dict
        # model["reward_report"] = reward_report
        # model["action_num_report"] = action_num_report
        # model["trajectory_memory"] = trajectory_memory
        # torch.save(model, 'Model_QLearning_HoldHandle_1_exploit_retrain.pt')

    node.get_logger().info("The training is finished, now the node is destroyed")
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()