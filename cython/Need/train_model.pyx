import sys
import os
sys.path.insert(0, os.path.abspath(os.path.join(os.getenv("HOME"), "Formula-Student-Driverless-Simulator/python")))
import fsds

import matplotlib.pyplot as plt

import qlearn
import SimEnv2 as SimEnv

from collections import deque
from CarRacingDQNAgent import CarRacingDQNAgent
from common_functions import process_state_image
from common_functions import generate_state_frame_stack_from_queue

plt.ion()
plt.show(block=False)

cpdef train(args):
    
    # connect to the simulator 
    #client = fsds.FSDSClient()

    # Check network connection, exit if not connected
    #client.confirmConnection()

    # After enabling setting trajectory setpoints via the api. 
    #client.enableApiControl(True)
    cdef bint RENDER                        = True
    cdef int  STARTING_EPISODE              = 1
    cdef int  ENDING_EPISODE                = 1000
    cdef int  SKIP_FRAMES                   = 2
    cdef int  SKIP_FRAMES_RENDERING         = 20
    cdef int  TRAINING_BATCH_SIZE           = 64
    cdef int  SAVE_TRAINING_FREQUENCY       = 1 # Save the model for every episode
    cdef int  UPDATE_TARGET_MODEL_FREQUENCY = 5
    cdef int  e
    cdef float total_reward
    cdef float CLOCK_SPEED = 0.08
    cdef int negative_reward_counter
    cdef int time_frame_counter
    cdef bint done 
    cdef float reward
    cdef int _

    env = SimEnv.Env()
    agent = CarRacingDQNAgent(epsilon=args.epsilon)
    if args.model:
        agent.load(args.model)
    if args.start:
        STARTING_EPISODE = args.start
    if args.end:
        ENDING_EPISODE = args.end
    
    
    for e in range(STARTING_EPISODE, ENDING_EPISODE+1):
        init_state = env.reset()
        init_state = process_state_image(init_state)

        total_reward = 0
        negative_reward_counter = 0
        state_frame_stack_queue = deque([init_state]*agent.frame_stack_num, maxlen=agent.frame_stack_num)
        time_frame_counter = 1
        done = False
        
        while True:
            if RENDER:
                env.render()

            current_state_frame_stack = generate_state_frame_stack_from_queue(state_frame_stack_queue)
            action = agent.act(current_state_frame_stack)

            reward = 0
            for _ in range(SKIP_FRAMES+1):
                next_state, r, done, info = env.step(action)

                if RENDER:
                    env.render()
                #plt.figure(1); plt.clf()
                #plt.imshow(next_state)
                #plt.title('state')
                #plt.draw()
                #plt.pause(0.001)
                
                reward += r
                if done:
                    break

            # If continually getting negative reward 10 times after the tolerance steps, terminate this episode
            negative_reward_counter = negative_reward_counter + 1 if time_frame_counter > 100 and reward < 0 else 0

            # Extra bonus for the model if it uses full gas
            if action[1] == 1 and action[2] == 0:
                reward *= 1.5

            total_reward += reward

            next_state = process_state_image(next_state)
            state_frame_stack_queue.append(next_state)
            next_state_frame_stack = generate_state_frame_stack_from_queue(state_frame_stack_queue)

            agent.memorize(current_state_frame_stack, action, reward, next_state_frame_stack, done)

            if done or negative_reward_counter >= 25 or total_reward < 0:
                print('Episode: {}/{}, Scores(Time Frames): {}, Total Rewards(adjusted): {:.2}, Epsilon: {:.2}'.format(e, ENDING_EPISODE, time_frame_counter, float(total_reward), float(agent.epsilon)))
                break
            if len(agent.memory) > TRAINING_BATCH_SIZE:
                agent.replay(TRAINING_BATCH_SIZE)
            time_frame_counter += 1

        if e % UPDATE_TARGET_MODEL_FREQUENCY == 0:
            agent.update_target_model()

        if e % SAVE_TRAINING_FREQUENCY == 0:
            agent.save('./save/trial_{}.h5'.format(e))

    env.close()