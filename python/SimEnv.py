import numpy as np
import sys
import os
import random
import time
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
    def __init__(self, client):
        self.client = client # FSDSClient()
        self.action_space = Space([np.linspace(-1,1,action_size), np.linspace(-1,1,action_size)])
        self.compute_track_boundaries()
        pass

    def reset(self):
        self.client.reset()
        self.compute_state() # np.zeros((state_grid_size, state_grid_size), dtype=np.uint8)
        # self.client
        return self.state.flatten()

    def compute_state(self):
        cones = self.find_cones()
        # self.reset()
        self.state = np.zeros((state_grid_size, state_grid_size), dtype=np.uint8)
        for cone in cones:
            x_index = cone['x'] + state_grid_size//2
            y_index = cone['y'] + state_grid_size//2

            if 0 <= x_index < state_grid_size and 0 <= y_index < state_grid_size:
                if distance(cone['x'], cone['y'], 0, 0):
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
        self.tc = TrackCompute(self.rf)
        self.tc.compute()


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
        car_state = self.client.getCarState()
        self.kinematics = self.client.simGetGroundTruthKinematics()
        self.gps_data = self.client.getGpsData()
        x = self.kinematics.position.x_val + self.rf.initial_position.x
        y = self.kinematics.position.y_val + self.rf.initial_position.y

        #print(car_state)
        
        res = 0
        done = False
        # TODO Compute reward and done

        cones = self.find_cones()
        
        self.tc.update_car_position(x, y, cones)
        self.tc.render()

        num_cones = len(cones)
        if num_cones < 2:
            res += -30
            done = True
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
        return new_state.flatten(), reward, done, info
    
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
