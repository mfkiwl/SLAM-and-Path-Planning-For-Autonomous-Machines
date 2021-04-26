import numpy as np
import sys
import os
from datetime import datetime
import random
import time
import subprocess
import signal
import logging
from threading import Timer
import math
from FSDSglobals import *
from Track import TrackCompute
sys.path.insert(0, os.path.abspath(os.path.join(os.getenv("HOME"), "Formula-Student-Driverless-Simulator/python")))
import fsds

class Space:
    def __init__(self, spaces_list):
        self.spaces_list = spaces_list
        pass

    def sample(self):
        res = []
        for r in self.spaces_list:
            #res.append(np.random.choice(r))
            res.append(random.choice(r))
        return res

class Env:
    # TODO Add velocity to state
    # TODO Make grid nxm not nxn
    def __init__(self, executable, track=TRACKS[0]):
        self.executable = executable

        # self.client = client # FSDSClient()
        self.action_space = Space([np.linspace(-1,1,action_size), np.linspace(-1,1,action_size)])
        #self.compute_track_boundaries()
        self.track = track

        # The list of log-lines shown in the operator web gui
        self.logs = []

        # The mission that the team should use
        self.mission = None

        # The unreal engine map that is loaded when starting the simulator.
        self.track = None

        # The process descriptor of the simulator
        self.simulation_process = None

        # For every simulation launch, a new logfile is created and referee state (like lap times and DOO cones ) are stored.
        # This is the file object for the current run.
        self.log_file = None

        # Wether or not competition mode is enabled or disabled (boolean)
        #self.competition_mode = None 

        # Wether or not the finished signal is received
        self.finished_signal_received = False

        # The rpc client connected to the simulation. Used to retrieve referee state.
        self.client = None
        
        # The timer that triggers periodic referee state updates.
        self.referee_state_timer = None

        self.reset_message = ""
        pass

    def log(self, line):
        line = str(datetime.now()) + ": " + line
        #self.logs.append(line)
        #self.log_file.write(line + '\n')
        #print(line, flush=True)

    def launch_simulator(self):
        # Abort if simulator is already running
        if self.simulation_process is not None:
            print(400, 'Simulation already running.')

        # Get team config
        self.mission = MISSIONS[0] # request.json['mission']
        self.track = TRACKS[0] # request.json['track']
        
        # Create log file. Create logs directory if it does not exist
        filename = 'logs/{}_{}_{}.txt'.format(str(datetime.now().strftime("%d-%m-%Y_%H%M%S")), "DDQ", self.mission)
        simfilename = 'logs/{}_{}_{}_SIM.txt'.format(str(datetime.now().strftime("%d-%m-%Y_%H%M%S")), "DDQ", self.mission)
        if not os.path.exists(os.path.dirname(filename)):
            try:
                os.makedirs(os.path.dirname(filename))
            except OSError as exc: # Guard against race condition
                if exc.errno != errno.EEXIST:
                    raise
        # Write to log file
        self.log_file = open(filename, 'w')
        self.log("created logfile " + filename)

        self.finished_signal_received = False

        # Write team specific car settings to settings.json
        #filename = os.path.realpath(os.path.dirname(__file__)) + '/../settings.json'
        #with open(filename, 'w') as file:
        #    json.dump(self.team['car_settings'], file, sort_keys=True, indent=4, separators=(',', ': '))

        proc = None
        try:
            # Launch Unreal Engine simulator
            #proc = subprocess.Popen(['../simulator/FSDS.exe', '/Game/'+self.track+'?listen'])
            simfile = open(simfilename, 'w')
            proc = subprocess.Popen([self.executable, '/Game/'+self.track+'?listen'], stdout=simfile)

            time.sleep(10)

            # Create connection with airsim car client
            #self.client = fsds.CarClient()
            self.client = fsds.FSDSClient()
            self.client.confirmConnection()
            self.client.enableApiControl(True)

            self.compute_track_boundaries()

            # Get referee state
            ref =  self.client.getRefereeState()
            self.doo_count = ref.doo_counter
            self.lap_times = ref.laps

            # Start referee state listener
            self.referee_state_timer = Timer(1.5, self.referee_state_listener)
            self.referee_state_timer.start()

            self.simulation_process = proc

            self.log('Launched simulator. {} {} {}'.format("DDQ", self.track, self.mission))
            
            return {}  
        except Exception as e:
            #e = sys.exc_info()[0]
            print("Error while launching simulator", e)
            print(e)
            self.shutdown_process(proc)
            exit(1)
            raise

    def exit_simulator(self):
        # Abort if simulator is not running
        if self.simulation_process is None:
            print(400, 'Simulation not running.')
            return

        if self.client is not None:
            print("CLIENT RESET")
            self.client.reset()
            #time.sleep(1)

        # Close airsim client connection
        self.client = None
        self.referee_state_timer = None

        # Shutdown simulation processes
        self.shutdown_process(self.simulation_process)
        self.simulation_process = None
    
        self.log('Exited simulator.')
        
        return {}  

    def get_config(self):
        return {
            'team': "DDQ",
            'mission': self.mission,
            'track': self.track,
            #'competition_mode': self.competition_mode,
        }

    def shutdown_process(self, proc):
        if proc is None:
            return
        if proc.poll() is None:
            #print("Sim shutdown")
            # process has not (yet) terminated. 
            
            # Try to stop it gracefully.
            #os.kill(proc.pid, signal.CTRL_BREAK_EVENT)
            os.kill(proc.pid, signal.SIGINT)
            #time.sleep(5)
            
            # Still running?
            if proc.poll() is None:
                # Kill it with fire
                proc.terminate()
                # Wait for it to finish
                proc.wait()
            
            # Going to kill all related processes created by simulation_process
            #os.system("taskkill /im Blocks* /F")
            os.system("killall Blocks")

    def poll_server_state(self):

        return {
            'logs': self.logs,
            'simulator_state': True if self.simulation_process is not None else False,
        }

    def finished(self):
        if self.finished_signal_received:
            return {}
        self.log("Received finished signal from autonomous system.")
        self.finished_signal_received = True
        return {}

    def referee_state_listener(self):
        if self.referee_state_timer is None:
            return
        self.referee_state_timer = Timer(1.5, self.referee_state_listener)
        self.referee_state_timer.start()
        try:
            ref = self.client.getRefereeState()

            if self.doo_count != ref.doo_counter:
                delta = ref.doo_counter - self.doo_count

                for d in range(self.doo_count + 1, self.doo_count + delta + 1):
                    self.log('Cone hit. {} cone(s) DOO.'.format(d))
                    
                self.doo_count = ref.doo_counter

            if len(self.lap_times) != len(ref.laps):
                self.lap_times = ref.laps
                lap_count = len(self.lap_times)
                lap_time = self.lap_times[-1]
                self.log('Lap ' + str(lap_count) + ' completed. Lap time: ' + str(round(lap_time, 3)) + ' s.')
        except:
            pass


    def reset(self, message):
        print("Sim Reset : ", message, " : ", self.reset_message)
        #self.client.reset()
        #self.compute_state() # np.zeros((state_grid_size, state_grid_size), dtype=np.uint8)
        # self.client
        # return self.state.flatten()
        self.reset_message = ""
        self.exit_simulator()
        time.sleep(3)
        self.launch_simulator()
        return self.state

    def compute_state(self):
        cones = self.find_cones()
        # TODO : Generate image with cones
        self.state = np.zeros((state_grid_size, state_grid_size), dtype=np.uint8)
        for cone in cones:
            x_index = state_grid_size - ( cone['x'] + state_grid_size//2 )
            y_index = state_grid_size - ( cone['y'] + state_grid_size//2 )

            if 0 <= x_index < state_grid_size and 0 <= y_index < state_grid_size:
                #if distance(cone['x'], cone['y'], 0, 0):
                self.state[int(x_index), int(y_index)] = 1
        return self.state

    def compute_track_boundaries(self):

        # TODO Compute track path from RefereeState
        # TODO Graph out track and cones with colour
        # TODO Live position of car in track
        """
        <RefereeState> {
            'cones': [
                {
                    'color': 0,
                    'x': 5519.39501953125,
                    'y': 8775.1943359375
                }, ... 
            ],
            'doo_counter': 3,
            'initial_position': <Point2D> { 'x': 4575.15283203125, 'y': 8577.8154296875 },
            'laps': [ 4.393726348876953 ]
        }
        """
        self.rf = self.client.getRefereeState()
        self.state = self.compute_state()
        self.tc = TrackCompute(self.rf, self.state)
        self.tc.compute()


    def render(self):
        car_state = self.client.getCarState()
        self.kinematics = self.client.simGetGroundTruthKinematics()
        self.gps_data = self.client.getGpsData()
        x = self.kinematics.position.x_val + self.rf.initial_position.x
        y = self.kinematics.position.y_val + self.rf.initial_position.y
        
        #x = car_state.kinematics_estimated.position.x_val + self.rf.initial_position.x
        #y = car_state.kinematics_estimated.position.y_val + self.rf.initial_position.y
        cones = self.find_cones()
        
        self.tc.update_car_position(x, y, cones)
        
        self.state = self.compute_state()
        imgL, imgR, imgD = self.get_images()
        self.tc.render(self.state, imgL, imgR, imgD)

    def get_images(self):
        #z = np.zeros((10,10))
        #return z, z, z
        responses = self.client.simGetImages([
            fsds.ImageRequest("cam1", fsds.ImageType.Scene, False, False),
            fsds.ImageRequest("cam2", fsds.ImageType.Scene, False, False),
            fsds.ImageRequest("cam3", fsds.ImageType.DepthPlanner, pixels_as_float = True, compress=False)
        ])
        
        response = responses[0]
        # get np array
        img1d = np.fromstring(response.image_data_uint8, dtype=np.uint8) 
        # reshape array to 4 channel image array H X W X 4
        imgL = img1d.reshape(response.height, response.width, 3)

        #imgL =cv2.resize(imgL, ( imgL.shape[1]//2, imgL.shape[0]//2 ))
        # original image is fliped vertically
        #imgL = np.flipud(img_rgb)

        response = responses[1]
        # get np array
        img1d = np.fromstring(response.image_data_uint8, dtype=np.uint8)
        # reshape array to 4 channel image array H X W X 4
        imgR = img1d.reshape(response.height, response.width, 3)
        #imgR =cv2.resize(imgR, ( imgR.shape[1]//2, imgR.shape[0]//2 ))


        response = responses[2]
        # get np array
        # print(response.image_data_float)
        #print(dir(response))
        #img1d = np.fromstring(response.image_data_uint8, dtype=np.uint8) 
        
        img1d = np.uint8(response.image_data_float)
        # img1d = np.array(response.image_data_float)
        # reshape array to 4 channel image array H X W X 4
        imgD = img1d.reshape(response.height, response.width, 1)
        #imgD = cv2.resize(imgD, ( imgD.shape[1]//2, imgD.shape[0]//2 ))
        
        #imgLg = cv2.cvtColor(imgL, cv2.COLOR_BGR2GRAY)
        #imgRg = cv2.cvtColor(imgR, cv2.COLOR_BGR2GRAY)

        return imgL, imgR, imgD

    def compute_reward(self):
        """<KinematicsState> {
            'angular_acceleration': <Vector3r> 
            'angular_velocity': <Vector3r> ,
            'linear_acceleration': <Vector3r> ,
            'linear_velocity': <Vector3r> ,
            'orientation': <Quaternionr> ,
            'position': <Vector3r> {   'x_val': 7.125288009643555, 'y_val': 0.3718554675579071, 'z_val': 0.2494993507862091}
        }
        """
        """
        <CarState> {   'gear': 0,
    'handbrake': False,
    'kinematics_estimated': <KinematicsState> {   'angular_acceleration': <Vector3r> {   'x_val': -0.011636244133114815,
    'y_val': 0.013664432801306248,
    'z_val': -7.922106988189626e-07},
    'angular_velocity': <Vector3r> {   'x_val': 0.00011646282655419782,
    'y_val': -0.0001731524826027453,
    'z_val': 1.0502494873776413e-08},
    'linear_acceleration': <Vector3r> {   'x_val': 5.199217412155122e-07,
    'y_val': 0.00021228201512712985,
    'z_val': 0.013618909753859043},
    'linear_velocity': <Vector3r> {   'x_val': -6.962161023693625e-06,
    'y_val': -2.0039804439875297e-05,
    'z_val': -0.0011192987440153956},
    'orientation': <Quaternionr> {   'w_val': 1.0,
    'x_val': 3.0327300919452682e-05,
    'y_val': -2.318620499863755e-05,
    'z_val': 7.031750182129315e-10},
    'position': <Vector3r> {   'x_val': 1.4648437172581907e-05,
    'y_val': 9.765624781721272e-06,
    'z_val': 0.2462308406829834}},
    'maxrpm': 13000.0,
    'rpm': 0.0,
    'speed': -7.0243090704025235e-06,
    'timestamp': 1616825965530656000}
        """
        car_state = self.client.getCarState()
        self.kinematics = self.client.simGetGroundTruthKinematics()
        self.gps_data = self.client.getGpsData()
        x = self.kinematics.position.x_val + self.rf.initial_position.x
        y = self.kinematics.position.y_val + self.rf.initial_position.y
        
        x = car_state.kinematics_estimated.position.x_val + self.rf.initial_position.x
        y = car_state.kinematics_estimated.position.y_val + self.rf.initial_position.y
        #print(x, y)
        #print(car_state)
        
        res = 0
        done = False
        # TODO Compute reward and done

        cones = self.find_cones()
        
        self.tc.update_car_position(x, y, cones)

        num_cones = len(cones)
        if num_cones < 2:
            res += -30
            done = True
            self.reset_message += " Less than 2 cones visible "
            return res, done
        # Mean of x axis should be close to zero
        x_sum = 0
        for cone in cones:
            x_sum += cone['x']
        x_avg = x_sum / num_cones
        
        res_x_avg = 10
        if x_avg!=0:
            res_x_avg = -1 * math.log(abs(x_avg))
        
        if res_x_avg > 10:
            res_x_avg = 10

        if res_x_avg < -10:
            res_x_avg = -10

        # res += res_x_avg
        # Count the number of cones

        sorted_cones = sorted(cones, key=lambda cone: cone['y'])
        left_cones = sorted_cones[:num_cones//2]
        right_cones = sorted_cones[num_cones//2:]
        if num_cones%2!=0:
            mid_left = sorted_cones[(num_cones)//2 - 1]
            middle_cone = sorted_cones[num_cones//2]
            mid_right = sorted_cones[(num_cones)//2 + 1]
            
            left_cones = sorted_cones[:num_cones//2]
            right_cones = sorted_cones[(num_cones//2)+1:]
            
            if abs(mid_left['y'] - middle_cone['y']) < abs(mid_right['y'] - middle_cone['y']):
                left_cones.append(middle_cone)
            else:
                right_cones.append(middle_cone)

        cone_reward = 0
        for cone in left_cones:
            if cone['y']>0:
                cone_reward += 1
            else:
                cone_reward += -10
                # done = True
        
        for cone in left_cones:
            if cone['y']>0:
                cone_reward += 1
            else:
                cone_reward += -10
                # done = True
        

        # print(dir(self.client))
        self.collisions = self.client.client.call("simGetCollisionInfo", 'FSCar')

        # print(self.collisions)
        # {'has_collided': False, 'penetration_depth': 0.0, 'time_stamp': 0, 'normal': {'x_val': 0.0, 'y_val': 0.0, 'z_val': 0.0}, 'impact_point': {'x_val': 0.0, 'y_val': 0.0, 'z_val': 0.0}, 'position': {'x_val': 0.0, 'y_val': 0.0, 'z_val': 0.0}, 'object_name': '', 'object_id': -1}

        if self.collisions["has_collided"]:
            print("========collision=========")
            print(self.collisions)
            res += -50
            done = True
            self.reset_message += " Collision "
            
        res += car_state.speed / 3 
        #if car_state.speed>0 and car_state.speed<0.9999:
        #    res += -1 * math.log(1 - car_state.speed) / 4
        #elif (car_state.speed>=1):
        #    res += 1.2
        return res, done

    def step(self, action):
        # TODO Check if action is in action_space
        # TODO return info - debug information

        car_controls = fsds.CarControls()
        car_controls.steering = action[0]
        car_controls.throttle = 0
        car_controls.brake = 0
        if action[1]>0:
            car_controls.throttle = action[1]
        else:
            car_controls.brake = action[1] * -1
        self.client.setCarControls(car_controls)

        # time.sleep(0.3)

        new_state = self.compute_state()

        reward, done = self.compute_reward()
        info = self.client.getCarState()
        #return new_state.flatten(), reward, done, info
        return new_state, reward, done, info
    
    def find_cones(self):
        # Get the pointcloud
        lidardata = self.client.getLidarData(lidar_name = 'Lidar')

        # no points
        if len(lidardata.point_cloud) < 3:
            return []

        # Convert the list of floats into a list of xyz coordinates
        points = np.array(lidardata.point_cloud, dtype=np.dtype('f4'))
        points = np.reshape(points, (int(points.shape[0]/3), 3))

        # Go through all the points and find nearby groups of points that are close together as those will probably be cones.

        current_group = []
        cones = []
        for i in range(1, len(points)):

            # Get the distance from current to previous point
            distance_to_last_point = distance(points[i][0], points[i][1], points[i-1][0], points[i-1][1])

            if distance_to_last_point < 0.1:
                # Points closer together then 10 cm are part of the same group
                current_group.append({'x': points[i][0], 'y': points[i][1]})
            else:
                # points further away indiate a split between groups
                if len(current_group) > 0:
                    cone = pointgroup_to_cone(current_group)
                    # calculate distance between lidar and cone
                    if distance(0, 0, cone['x'], cone['y']) < cones_range_cutoff:
                        cones.append(cone)
                    current_group = []
        return cones
