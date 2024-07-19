#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
import torch

'''import policy algorithm'''
from MonteCarloGLIE import MonteCarloControlGLIE
from nStepTD import nStepTD
from Q_learning import QLearning

'''import environment'''
from DoorTraversal_env import DoorTraversalEnv
from HoldHandle_env import HoldHandleEnv

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

    env = DoorTraversalEnv()
    if node._training_mode == "training":
        f = 'test2.pt'
        episode = 6000
        print("Training for ", episode, " episodes")
        print("Saving the model to ", f)
        print("----------------------Training the model----------------------")
        MonteCarloControlGLIE(env=env,seed=0, num_episodes=episode, model = None, epsilon=1.0,filename=f)

    elif node._training_mode == "retraining":
        f = 'EX_DoorTraversal_MC_6000_retrain15.pt'
        model = torch.load('EX_DoorTraversal_MC_6000_retrain14.pt')

        episode = 145
        print("Training for ", episode, " episodes")
        print("Saving the model to ", f)
        print("----------------------Training the model----------------------")
        MonteCarloControlGLIE(env=env,seed=0, num_episodes=episode, model=model,epsilon=1.0,filename=f)

    node.get_logger().info("The training is finished, now the node is destroyed")
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()