import numpy as np
import random
import time
from FSDSglobals import *

rewards = []

def max_d(d):
    res = False
    for key in d:
        if res != False:
            if d[key] > res:
                res = d[key]
        else:
            res = d[key]
    return res


def max_d_a(d):
    res = False
    res_a = False
    for key in d:
        if res != False:
            if d[key] > res:
                res = d[key]
                res_a = key
        else:
            res = d[key]
            res_a = key
    res_a = res_a.replace("[", "").replace("]", "")
    res_a = list(map(float, res_a.split(",")))
    return res_a

def start_learning(env):
    epsilon = 1.0
    # qtable = np.zeros((state_size, action_size))
    qtable = dict()
    # 2 For life or until learning is stopped
    for episode in range(total_episodes):
        # Reset the environment
        state = env.reset()
        step = 0
        done = False
        total_rewards = 0
        
        for step in range(max_steps):
            # 3. Choose an action a in the current world state (s)
            ## First we randomize a number
            exp_exp_tradeoff = random.uniform(0, 1)
            qtable.setdefault(str(state), dict())

            for i in np.linspace(-1,1,action_size):
                for j in np.linspace(-1,1,action_size):
                    qtable[str(state)].setdefault(str([i, j]), 0.0)
            ## If this number > greater than epsilon --> exploitation (taking the biggest Q value for this state)
            if exp_exp_tradeoff > epsilon:
                # action = np.argmax(qtable[str(state)])
                action = max_d_a(qtable[str(state)])

            # Else doing a random choice --> exploration
            else:
                action = env.action_space.sample()

            # Take the action (a) and observe the outcome state(s') and reward (r)

            new_state, reward, done, info = env.step(action)
            
            # print(time.time(), info, action, reward, sep="\t")
            qtable.setdefault(str(new_state), dict())

            for i in range(-1, 1, action_size):
                for j in range(-1, 1, action_size):
                    qtable[str(new_state)].setdefault(str([i, j]), 0.0)
            
            # Update Q(s,a):= Q(s,a) + lr [R(s,a) + gamma * max Q(s',a') - Q(s,a)]
            # qtable[new_state,:] : all the actions we can take from new state
            # qtable[state, action] = qtable[state, action] + learning_rate * (reward + gamma * np.max(qtable[new_state, :]) - qtable[state, action])
            qtable[str(state)][str(action)] = qtable[str(state)][str(action)] + learning_rate * (reward + gamma * max_d(qtable[str(new_state)]) - qtable[str(state)][str(action)])
            total_rewards += reward
            
            # Our new state is state
            state = new_state
            
            # If done (if we're dead) : finish episode
            if done == True:
                break
            
        # Reduce epsilon (because we need less and less exploration)
        epsilon = min_epsilon + (max_epsilon - min_epsilon)*np.exp(-decay_rate*episode) 
        rewards.append(total_rewards)
        print("===========reset==============")
        # TODO : Save Episode with number

    print ("Score over time: " +  str(sum(rewards)/total_episodes))
    print(qtable)

    numpy.save("qtable.npy", qtable)
