import numpy as np
import sys
import os
from FSDSglobals import *

sys.path.insert(0, os.path.abspath(os.path.join(os.getenv("HOME"), "Formula-Student-Driverless-Simulator/python")))
import fsds

class Space:
    def __init__(self, spaces_list):
        self.spaces_list = spaces_list
        pass

    def sample(self):
        res = []
        for r in self.spaces_list:
            res.append(np.random.choice(r))
        return res

class Env:
    # TODO Add velocity to state
    # TODO Make grid nxm not nxn
    def __init__(self, client):
        self.client = client # FSDSClient()
        self.action_space = Space([range(-1, 1, action_size), range(-1, 1, action_size)])
        pass

    def reset(self):
        # TODO Reset FSDS
        self.state = np.zeros((state_grid_size, state_grid_size), dtype=numpy.int8)
        self.client
        return self.state

    def compute_state(self):
        cones = find_cones(self)
        reset(self)

        for cone in cones:
            x_index = cone['x'] + state_grid_size//2
            y_index = cone['y'] + state_grid_size//2

            if 0 <= x_index < state_grid_size and 0 <= y_index < state_grid_size:
                if distance(cone['x'], cone['y'], 0, 0):
                    self.state[x_index][y_index] = 1
        return state

    def compute_reward(self):
        res = 0
        done = False
        # TODO Compute reward and done
        return res, done

    def step(self, action):
        # TODO Check if action is in action_space
        # TODO return info

        car_controls = fsds.CarControls()
        car_controls.steering = action[0]
        car_controls.throttle = 0
        car_controls.brake = 0
        if action[1]>0:
            car_controls.throttle = action[1]
        else:
            car_controls.brake = action[1] * -1
        self.client.setCarControls(car_controls)

        time.sleep(1)

        new_state = compute_state(self)

        reward, done = compute_reward(self)

        return new_state, reward, done, info
    
    def find_cones(self):
        # Get the pointcloud
        lidardata = self.client.getLidarData(lidar_name = 'Lidar')

        # no points
        if len(lidardata.point_cloud) < 3:
            return []

        # Convert the list of floats into a list of xyz coordinates
        points = numpy.array(lidardata.point_cloud, dtype=numpy.dtype('f4'))
        points = numpy.reshape(points, (int(points.shape[0]/3), 3))

        process_lidar(points)
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
