#!/usr/bin/env python3

import numpy as np
import torch
import random
import pickle
from collections import defaultdict

device = torch.device('cuda:0' if torch.cuda.is_available() else 'cpu')
state_table = torch.zeros(1,2,2,3,2) #(2,2,2,2,2,140,2,2,2,2,2,2)
action_num = 7
max_idx_action = 0
retrain = 0

def n_step_TD(env, seed, episodes=1000, model=None, epsilon=1.0,filename='model.pt'):
    
    returns = defaultdict(lambda: {'G':0.0, 'N':0.0} )
    init_BT_string = str([])
    update_Q_memory = defaultdict(lambda: {'trajec_max':[], 'reward_max':-100.0, 'Q_num':0.0} )

    if not model:
        # policy = create_random_policy(state_table,action_num)
        policy = create_policy()
        Q = {init_BT_string:policy}
        BT_string_dict = {}
        reward_report = []
        action_num_report = []
        trajectory_memory = []
        return_memory = []
    else:
        Q = model["policy"]
        returns_old = model["returns"]
        for key in returns_old.keys():
            returns[key] = returns_old[key]
        BT_string_dict = model["BT_string_dict"]
        reward_report = model["reward_report"]
        action_num_report = model["action_num_report"]
        trajectory_memory = model["trajectory_memory"]
        return_memory = model["return_memory"]
    
    '''for retraining mode: must load learned returns and BT_string_dict'''
    for i_episode in range(episodes):
        # Q_init = create_random_policy(state_table,action_num)

        Q_init = create_policy()
        episode_SAR, Q_new, BT_string_dict,returns,episode_reward,bt_length,trajectory,episode_return = run_BT(env,seed,Q,epsilon,BT_string_dict,Q_init,returns) # Store state[Dict()], action[List] and reward[] respectively from start until terminate
        print("len episode a:",len(episode_SAR))
        
        Q = cal_G(episode_SAR,returns,Q_new,update_Q_memory,trajectory,episode_reward)

        '''update epsilon'''
        lr = 0.5 
        if (i_episode+retrain) < ((episodes+retrain)*0.7):
            lr = 6.0
        else:
            lr = 4.0
        
        print("lr:",lr)

        epsilon = 1/(((9.0*(i_episode+retrain))/(lr*(episodes+retrain)))+1)
        print("i_episode:",i_episode)
        print("epsilon:",epsilon)
        # if epsilon < 0.2:
        #     epsilon = 0.2

        if i_episode == episodes-1:
            epsilon = 0.0
        # epsilon = 0.0
        
            
        print("episode_reward: ",episode_reward)

        reward_report.append(episode_reward)
        action_num_report.append(bt_length)
        trajectory_memory.append(trajectory)
        return_memory.append(episode_return)

        model = dict()
        model["policy"] = Q
        model["returns"] = dict(returns)
        model["BT_string_dict"] = BT_string_dict
        model["reward_report"] = reward_report
        model["action_num_report"] = action_num_report
        model["trajectory_memory"] = trajectory_memory
        model["return_memory"] = return_memory
        torch.save(model, filename)
        print("save model!!")

        print("-------------End episodes-------------")

    return Q,returns,BT_string_dict,reward_report,action_num_report,trajectory_memory,return_memory

def cal_G(episode_SAR,returns,Q_new,update_Q_memory,trajectory,episode_reward):
    G = 0.0
    discount_factor = 0.3
    Q = Q_new.copy()
    state_list = []

    for i in reversed(range(0, len(episode_SAR))):
        [s_t, a_t, r_t_1] = episode_SAR[i] 

        v = 0.0
        key_list = get_state(s_t)

        currentQ_dict = Q_new[s_t["BT_string"]][key_list[0]][key_list[1]][key_list[2]][key_list[3]][key_list[4]]
        keys = currentQ_dict.keys()
        
        for key in keys:
            v += currentQ_dict[key]
        '''calculate G'''
        G = discount_factor * v + r_t_1

        state_action = state_action_str(s_t, a_t) # list of state_action
        # state_action = str(state_action) ## change to -> [1,0,0,0,....,(2,0)]
        is_first_visit = True

        for j in range(0,i):

            if tuple(episode_SAR[j][0]) == tuple(s_t) and tuple(episode_SAR[j][1]) == tuple(a_t):
                is_first_visit = False
                break

        '''check first visit in the episode'''
        if is_first_visit:
            '''update returns'''
            returns[state_action]['N'] += 1
            returns[state_action]['G'] += G

            '''update Q table'''
            Q_new = update_Q(Q_new,s_t,a_t,returns[state_action])
    Q = Q_new

    if trajectory != None: 
        for l in reversed(range(0, len(trajectory))):
            [s_t, a_t, r_t_1] = trajectory[l] 
            is_first_visit = True
            key_list = get_state(s_t)

            if is_first_visit:
                '''update Q_memory'''
                state_list.append(s_t["BT_string"])
                for k in range(5):
                    state_list.append(key_list[k])
                state_tuple = tuple(state_list)
                # print("!!!!! Add Q mem:", state_tuple)

                update_Q_memory[state_tuple]["Q_num"] += 1
                if episode_reward[0] > update_Q_memory[state_tuple]["reward_max"]:
                    update_Q_memory[state_tuple]["reward_max"] = episode_reward[0]
                    update_Q_memory[state_tuple]["trajec_max"] = trajectory
                    # print("!!!!! Buffer trajec max:", state_tuple)

                    if update_Q_memory[state_tuple]["Q_num"]%31 == 30:
                        ##update Q table
                        discount_update = 0.85
                        print("!!!!! Update Q mem:", state_tuple)

                        for m in reversed(range(l, len(update_Q_memory[state_tuple]["trajec_max"]))):
                            [s_t, a_t, r_t_1] = update_Q_memory[state_tuple]["trajec_max"][m] 

                            '''calculate G'''
                            G = discount_update * G + r_t_1
                            if s_t["BT_string"] == state_tuple[0]:
                                returns_dict = {'G':G, 'N':1.0}
                                Q = update_Q(Q_new,s_t,a_t,returns_dict)
                                break

    return Q

def create_random_policy(state_table, action_num):
    policy = {}
    n_action = action_num

    for action in range(n_action):
        policy[(action,0)] = 0.0

    for i in reversed(range(1, len(state_table.shape))): #for i in reversed(range(len(state_table.shape)))
        n_state = state_table.shape[i]
        p = policy_init(n_state, policy)
        policy = p

    '''especially for hold handle subtask'''
    new_policy = {}
    new_policy[1] = policy
    policy = new_policy

    return policy

def create_policy():
    robot_pose_key = ["front_door","find_aruco","approach_door"]
    gripper_pose_key = ["home_pose","move_linear"]

    Dict_test = dict()
    for a in range(2): # have door path state
        Dict_test[a] = dict() 
        for b in range(2): # gripper state
            Dict_test[a][b] = dict()
            for c in robot_pose_key: # robot pose state
                Dict_test[a][b][c] = dict()
                for d in gripper_pose_key: # gripper pose state
                    Dict_test[a][b][c][d] = dict()
                    for i in range(action_num):
                        Dict_test[a][b][c][d][(i,0)] = 0.0
    
    policy = dict()
    policy[0] = Dict_test # find aruco state
    policy[1] = Dict_test

    return policy

def policy_init(n_state,p):
    policy = {}
    for key in range(0, n_state):
        policy[key] = p
    return policy

def run_BT(env,seed,Q_table,epsilon,BT_string_dict,Q_init,returns): ## Q_init is changed to have value equal to Q_update 
    done = False
    episode_SAR = []
    trajectory = []
    initial_state, _ = env.reset()
    state = initial_state
    # cumulative_reward = 0
    max_idx_action = 1
    Q = Q_table
    
    old_action = -1
    old_index_action = -1
    step_num = 0
    bt_length = 0
    is_exploit_list = []
    episode_return = [0.0,0.0,0.0,0.0,0.0,0.0]

    while not done:
        step_num += 1
        time_step = []
        time_step.append(state)

        # action_num, idx_action_position, Q_table = choose_action(Q,state,epsilon,max_idx_action,old_action,old_index_action)
        action_num, idx_action_position, Q_table, is_exploit = choose_action(seed,Q,state,epsilon,max_idx_action,old_action,old_index_action)

        state, reward, done, _,subtree_num,bt_length,episode_return = env.step(action_num=action_num, idx_action_position=idx_action_position,returns=episode_return)
        
        old_action = action_num
        old_index_action = idx_action_position
        # if BT_string_dict.get(state["BT_string"]) is None: # BT_string collection
        print("BT_string:",state["BT_string"])
        BT_string_dict[state["BT_string"]] = state["BT_string"]

        if Q_table.get(state["BT_string"]) is None:
            Q_table[state["BT_string"]] = Q_init # add BT_string to state-action table # Q_table
        
        print("subtree_num:",subtree_num)
        max_idx_action = len(subtree_num)
        # print("max_idx_action:",max_idx_action)

        # cumulative_reward += reward
        
        action = [action_num, idx_action_position]
        time_step.append(action)
        time_step.append(reward)
        episode_SAR.append(time_step)
        trajectory.append(time_step)

        if step_num%3 == 0:
            Q = cal_G(episode_SAR,returns,Q_table,update_Q_memory=None,trajectory=None,episode_reward=None)
            Q_table = Q
            episode_SAR = []
            print("Update Q table!!")

        is_exploit_list.append(is_exploit)

        print("*-----------End time step-----------*")

    Q = Q_table
    episode_reward = (reward,is_exploit_list)
    return episode_SAR, Q, BT_string_dict,returns,episode_reward,bt_length,trajectory,episode_return

def choose_action(seed,Q,state,epsilon,max_idx_action,old_action,old_index_action):
    """
    Choose an action based on the epsilon-greedy.
    output: action_num, the index of the collection_position
    """
    num = 7
    key_list = get_state(state)
    print("key_list:",key_list)
    print("BT_string:",state["BT_string"])
    # print("Q:",Q)

    Q_state = Q[state["BT_string"]][key_list[0]][key_list[1]][key_list[2]][key_list[3]][key_list[4]].copy()

    for i in range(max_idx_action):
        if Q_state.get((0,i)) is None:
            # print("Add action to Q table")
            for j in range(num):
                Q[state["BT_string"]][key_list[0]][key_list[1]][key_list[2]][key_list[3]][key_list[4]][(j,i)] = 0.0
                Q_state[(j,i)] = 0.0

    over_idx_position = max_idx_action
    # Q_state_enable = Q_state.copy()
    while True:
        if Q_state.get((0,over_idx_position)) is not None:
            for k in range(num):
                Q_state[(k,over_idx_position)] = -1000
            over_idx_position += 1
        else:
            break

    action_num = -1
    idx_action_position = -1
    np.random.seed(seed)
    while True:
        is_exploit = 0
        random_num = np.random.random()
        if random_num <= epsilon:
            '''exploration'''
            print("exploration!!")
            action_num = random.randint(0,num-1)
            idx_action_position = random.randint(0,max_idx_action-1)
        else:
            '''exploitation'''
            print("exploitation!!")
            action_num, idx_action_position = max(Q_state, key=Q_state.get)
            is_exploit = 1

        if old_action!=action_num or old_index_action!=idx_action_position or epsilon==0.0:
            break

    # action_num, idx_action_position = max(Q_state, key=Q_state.get)
        
    return action_num, idx_action_position, Q, is_exploit

def state_action_str(state,action):
    state_action = ""
    for key in state.keys():
        state_action = state_action+str(state[key])
    state_action = state_action+str(tuple(action))
    return state_action

def update_Q(Q,state,action,returns_dict):
    cumulative_G = returns_dict["G"]
    N = returns_dict["N"]
    avg_G = cumulative_G / N

    key_list = get_state(state)

    Q_update = Q

    # print('Q update: ',Q_update[state["BT_string"]][key_list[0]][key_list[1]][key_list[2]][key_list[3]][key_list[4]])
    # print('action: ',action)
    Q_update[state["BT_string"]][key_list[0]][key_list[1]][key_list[2]][key_list[3]][key_list[4]][tuple(action)] = avg_G
    
    return Q_update

def get_state(state):
    find_aruco = state["find_aruco_status"][0]
    have_door_path = state["have_door_path"][0]
    gripper_status = state["gripper_status"][0]
    robot_x = state["robot_pose"][0]
    robot_y = state["robot_pose"][1]
    robot_theta = float(state["robot_pose"][2])

    gripper_x = state["gripper_base_pose"][0]
    gripper_y= state["gripper_base_pose"][1]
    gripper_z = state["gripper_base_pose"][2]
    gripper_R = state["gripper_base_pose"][3]
    gripper_P = state["gripper_base_pose"][4]
    gripper_Y = state["gripper_base_pose"][5]
    

    if round(float(robot_x),1) == -2.0 and round(float(robot_y),1) == -1.5 and round(robot_theta,1) == -2.1:
        robot_pose_state = "front_door"
    elif round(float(robot_x),1) == -2.0 and round(float(robot_y),1) == -1.5 and round(robot_theta,1) == -1.8:
        robot_pose_state = "find_aruco"
    if round(float(robot_x),1) == -1.9 and round(float(robot_y),1) == -2.2 and round(robot_theta,1) == -1.4:
        robot_pose_state = "approach_door"

    # gripper_pose_state = "home_pose"

    if round(float(gripper_x),1) == 0.4 and round(float(gripper_y),1) == 0.1 and round(float(gripper_z),1) == 1.0 and round(float(gripper_R),1) == 1.5 and round(float(gripper_P),1) == 0.0 and round(float(gripper_Y),1) == 0.0:
        gripper_pose_state = "home_pose"
    elif round(float(gripper_x),1) == 0.8 and round(float(gripper_y),1) == 0.2 and round(float(gripper_z),1) == 1.0 and round(float(gripper_R),1) == 1.5 and round(float(gripper_P),1) == 0.0 and round(float(gripper_Y),1) == 0.2:   
        gripper_pose_state = "move_linear"

    # robot_x = check_state(state["robot_pose"][0],-2.0,0,-1.9,1)
    # robot_y = check_state(state["robot_pose"][1],-1.5,0,-2.2,1)
    # # print(state["gripper_base_pose"][0],", ",state["gripper_base_pose"][1])
    # gripper_x = check_state(state["gripper_base_pose"][0],0.4,0,0.8,1)
    # # print("gripper_x:", gripper_x)
    # gripper_y= check_state(state["gripper_base_pose"][1],0.1,0,0.2,1)
    # # print("gripper_y:", gripper_y)
    # gripper_z = check_state(state["gripper_base_pose"][2],1.0,0,1.0,0)
    # gripper_R = check_state(state["gripper_base_pose"][3],1.5,0,1.5,0)
    # gripper_P = check_state(state["gripper_base_pose"][4],0.0,0,0.0,0)
    # gripper_Y = check_state(state["gripper_base_pose"][5],0.0,0,0.2,1)
    # theta = float(state["robot_pose"][2])
    # robot_theta = 0
    # if round(theta,1) == -1.4:
    #     robot_theta = 0
    # elif round(theta,1) == -1.8:
    #     robot_theta = 1
    # elif round(theta,1) == -2.1:
    #     robot_theta = 2
    # print("Theta in get state:",robot_theta)
    # robot_theta = round((theta+3.5)/0.05)

    return [find_aruco,have_door_path,gripper_status,robot_pose_state,gripper_pose_state]

def nStepTD(env,seed, num_episodes, model, epsilon,filename):
    """
    GLIE Monte Carlo Control algorithm.
    """
    env = env
    num_episodes = num_episodes
    epsilon = epsilon

    policy,returns,BT_string_dict,reward_report,action_num_report,trajectory_memory,return_memory = n_step_TD(env=env,seed=seed, episodes=num_episodes, model=model, epsilon=epsilon,filename=filename)
    return policy,returns,BT_string_dict,reward_report,action_num_report,trajectory_memory,return_memory