import sys
import os
sys.path.insert(0, os.path.abspath(os.path.join(os.getenv("HOME"), "Formula-Student-Driverless-Simulator/python")))
import fsds

import matplotlib.pyplot as plt

import qlearn
import SimEnv2 as SimEnv

import time
import argparse
from collections import deque
from CarRacingDQNAgent import CarRacingDQNAgent
from common_functions import process_state_image
from common_functions import generate_state_frame_stack_from_queue

plt.ion()
plt.show(block=False)


RENDER                        = True
STARTING_EPISODE              = 1
ENDING_EPISODE                = 1000
SKIP_FRAMES                   = 5
SKIP_FRAMES_RENDERING         = 20
TRAINING_BATCH_SIZE           = 64
#SAVE_TRAINING_FREQUENCY       = 25
SAVE_TRAINING_FREQUENCY       = 1
UPDATE_TARGET_MODEL_FREQUENCY = 5

CLOCK_SPEED = 0.1

def get_latest_save():
    save_files = os.listdir("save/")
    save_files.sort()
    return os.path.join("save", save_files[-1])

if __name__ == '__main__':
    parser = argparse.ArgumentParser(description='Training a DQN agent to play CarRacing.')
    parser.add_argument('-m', '--model', help='Specify the last trained model path if you want to continue training after it.')
    parser.add_argument('-s', '--start', type=int, help='The starting episode, default to 1.')
    parser.add_argument('-e', '--end', type=int, help='The ending episode, default to 1000.')
    parser.add_argument('-p', '--epsilon', type=float, default=1.0, help='The starting epsilon of the agent, default to 1.0.')
    parser.add_argument('-exe', '--executable', type=str, default="../../Apps/fsds-v2.0.0-linux/FSDS.sh", help='Path to FSDS.sh')
    parser.add_argument('-r', '--resume', default=False, action='store_true', help='Will resume training from latest save')
    parser.add_argument('-n', '--normal', default=False, action='store_true', help='Run at realtime speed')
    parser.add_argument('-t', '--test', default=False, action='store_true', help='Drive the car with a model')
    args = parser.parse_args()


    if args.resume:
        args.model = get_latest_save()

    if args.test and not args.model:
        print("No model provided")

    if args.normal:
        CLOCK_SPEED = 1.0

    # connect to the simulator 
    #client = fsds.FSDSClient()

    # Check network connection, exit if not connected
    #client.confirmConnection()

    # After enabling setting trajectory setpoints via the api. 
    #client.enableApiControl(True)

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
            

            #t1 = time.time()
            current_state_frame_stack = generate_state_frame_stack_from_queue(state_frame_stack_queue)
            action = agent.act(current_state_frame_stack)
            #t2 = time.time()
            #print("agent.act", t2-t1)

            reward = 0
            for _ in range(SKIP_FRAMES+1):
                next_state, r, done, info = env.step(action)
                total_frame_counter += 1
                if RENDER and total_frame_counter%SKIP_FRAMES_RENDERING==0:
                    if args.normal: env.client.client.call("simPause", True)
                    env.render()
                    if args.normal: env.client.client.call("simPause", False)

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
            if action[1] > 0: reward += 5

            total_reward += reward
            
            #t1 = time.time()
            next_state = process_state_image(next_state)
            state_frame_stack_queue.append(next_state)
            next_state_frame_stack = generate_state_frame_stack_from_queue(state_frame_stack_queue)
            #t2 = time.time()
            #print("next_state", t2-t1)

            #t1 = time.time()
            if not args.test:
                agent.memorize(current_state_frame_stack, action, reward, next_state_frame_stack, done)
            #t2 = time.time()
            #print("agent.memorise", t2-t1)

            if done or negative_reward_counter >= 500 or total_reward < -20 and not args.test:
                if negative_reward_counter >= 500:
                    print("negative_reward_counter >= 500")
                if total_reward < -20:
                    print("total_reward < -20")
                end_time = time.time()
                FPS = total_frame_counter/(end_time - start_time) / CLOCK_SPEED
                print('Episode: {}/{}, FPS: {}, Scores(Time Frames): {}, Total Rewards(adjusted): {:.2}, Epsilon: {:.2}'.format(e, ENDING_EPISODE, FPS, time_frame_counter, float(total_reward), float(agent.epsilon)))
                break
            if len(agent.memory) > TRAINING_BATCH_SIZE and not args.test:
            
                #t1 = time.time()
                if args.normal: env.client.client.call("simPause", True)
                agent.replay(TRAINING_BATCH_SIZE)
                if args.normal: env.client.client.call("simPause", False)
                #t2 = time.time()
                #print("agent.replay", t2-t1)
            time_frame_counter += 1

        if e % UPDATE_TARGET_MODEL_FREQUENCY == 0 and not args.test:
            agent.update_target_model()

        if e % SAVE_TRAINING_FREQUENCY == 0 and not args.test:
            #agent.save('./save/trial_{:06d}.h5'.format(e))
            agent.save('./save/trial_{:06d}.h5'.format(int(time.time())))

    env.close()
