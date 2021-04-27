import sys
import os
sys.path.insert(0, os.path.abspath(os.path.join(os.getenv("HOME"), "Formula-Student-Driverless-Simulator/python")))
import fsds

import matplotlib.pyplot as plt

import qlearn
import SimEnv2 as SimEnv

import time
from collections import deque
from CarRacingDQNAgent import CarRacingDQNAgent
from common_functions import process_state_image
from common_functions import generate_state_frame_stack_from_queue

plt.ion()
plt.show(block=False)

def get_latest_save():
    save_files = os.listdir("save/")
    save_files.sort()
    return os.path.join("save", save_files[-1])

cpdef train(args):
    print(args)
    if args.resume:
        args.model = get_latest_save()

    # connect to the simulator 
    #client = fsds.FSDSClient()

    # Check network connection, exit if not connected
    #client.confirmConnection()

    # After enabling setting trajectory setpoints via the api. 
    #client.enableApiControl(True)
    cdef bint RENDER                        = True
    cdef bint done 
    cdef int  STARTING_EPISODE              = 1
    cdef int  ENDING_EPISODE                = 1000
    cdef int  SKIP_FRAMES                   = 2
    cdef int  SKIP_FRAMES_RENDERING         = 20
    cdef int  TRAINING_BATCH_SIZE           = 64
    cdef int  SAVE_TRAINING_FREQUENCY       = 1 # Save the model for every episode
    cdef int  UPDATE_TARGET_MODEL_FREQUENCY = 5
    cdef unsigned int  total_frame_counter # Number of frames run in that episode (used to calculate frame rate)
    cdef int  e
    cdef int  _
    cdef int negative_reward_counter
    cdef int time_frame_counter
    cdef float reward
    cdef double FPS
    cdef float total_reward
    cdef double start_time
    cdef double end_time
    cdef double delta
    cdef double t1
    cdef double t2
    cdef double CLOCK_SPEED = 0.08

    env = SimEnv.Env(args.executable)
    agent = CarRacingDQNAgent(epsilon=args.epsilon)
    if args.model:
        print("Resuming from ", args.model)
        agent.load(args.model)
    if args.start:
        STARTING_EPISODE = args.start
    if args.end:
        ENDING_EPISODE = args.end

    for e in range(STARTING_EPISODE, ENDING_EPISODE+1):
        init_state = env.reset("New Episode " + str(e))
        
        init_state = process_state_image(init_state)

        total_reward = 0
        negative_reward_counter = 0
        state_frame_stack_queue = deque([init_state]*agent.frame_stack_num, maxlen=agent.frame_stack_num)
        time_frame_counter = 1
        done = False
        
        start_time = time.time()
        total_frame_counter = 1
        
        while True:
            

            t1 = time.time()
            current_state_frame_stack = generate_state_frame_stack_from_queue(state_frame_stack_queue)
            action = agent.act(current_state_frame_stack)
            t2 = time.time()
            print("agent.act", t2-t1)

            reward = 0
            for _ in range(SKIP_FRAMES+1):
                next_state, r, done, info = env.step(action)
                total_frame_counter += 1
                if RENDER and total_frame_counter%SKIP_FRAMES_RENDERING==0: env.render()

                # if RENDER: env.render() # Don't render every frame

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
            
            t1 = time.time()
            next_state = process_state_image(next_state)
            state_frame_stack_queue.append(next_state)
            next_state_frame_stack = generate_state_frame_stack_from_queue(state_frame_stack_queue)
            t2 = time.time()
            print("next_state", t2-t1)

            t1 = time.time()
            agent.memorize(current_state_frame_stack, action, reward, next_state_frame_stack, done)
            t2 = time.time()
            print("agent.memorise", t2-t1)

            if done or negative_reward_counter >= 500 or total_reward < -20:
                if negative_reward_counter >= 500:
                    print("negative_reward_counter >= 500")
                if total_reward < -20:
                    print("total_reward < -20")
                end_time = time.time()
                delta = (end_time - start_time)
                if (delta > 0):
                    FPS = total_frame_counter/delta / CLOCK_SPEED
                else:
                    FPS = 999999
                print(start_time, end_time, total_frame_counter, CLOCK_SPEED)
                print('Episode: {}/{}, FPS: {}, Scores(Time Frames): {}, Total Rewards(adjusted): {:.2}, Epsilon: {:.2}'.format(e, ENDING_EPISODE, FPS, time_frame_counter, float(total_reward), float(agent.epsilon)))
                break
            if len(agent.memory) > TRAINING_BATCH_SIZE:
            
                t1 = time.time()
                agent.replay(TRAINING_BATCH_SIZE)
                t2 = time.time()
                print("agent.replay", t2-t1)
            time_frame_counter += 1

        if e % UPDATE_TARGET_MODEL_FREQUENCY == 0:
            agent.update_target_model()

        if e % SAVE_TRAINING_FREQUENCY == 0:
            #agent.save('./save/trial_{:06d}.h5'.format(e))
            agent.save('./save/trial_{:06d}.h5'.format(int(time.time())))

    env.close()