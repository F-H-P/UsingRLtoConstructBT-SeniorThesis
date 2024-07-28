# A Reinforcement Learning Framework for Autonomous Behavior Tree Construction in Door Traversal Tasks
---
This is the code of paper _**A Reinforcement Learning Framework for Autonomous Behavior Tree Construction in Door Traversal Tasks**_ that is implemented under Robot Operating System (ROS2) Humble framework.

## Code Usage

### Train the policy

There are two modes for training the policy: 1) "training" for training the new policy, and 2) "retraining" for training the trained policy. Four steps to run training process packing with:

#### **STEP1**: Set Arguements
In file "start_training.py" in _door_traversal package_, there are four arguements that require to set before training the policy.
* `'self._training_mode'` : the string argument to define training mode consist of **"training"** and **"retraining"**. 
* `'f'` : file name of learned policy that is saved while training until the training process is complete. 
* `'episode'` : the number of episodes that you want to train. 
* `'model'` : the model file that used to re-train. this argument is required when train in "retaining" mode only. 

#### **STEP2**: Build Environment

Make sure that all packages and any change in the workspace is builded completely.

`Command for build and source environment:`

```
$ cd ~/(path of the workspace)
```

```
$ colcon build && source install/setup.bash
```

#### **STEP3**: Run _GenBehaviorTree Node_

_GenBehaviorTree Node_ is the node of "bt_manager.py" in _mm_bt package_ that is used to manage generating the BT while training.

`Command:`

```
$ ros2 run mm_bt bt_manager.py
```

#### **STEP4**: Run _TrainingNode_

_TrainingNode_ is declared in file "start_training.py" and it is used to run whole RL training process in the system.

`Command:`

```
$ ros2 run door_traversal start_training.py
```
