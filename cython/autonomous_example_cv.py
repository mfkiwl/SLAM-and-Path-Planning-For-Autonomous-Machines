"""
This is a tiny autonomous system that should be able to finish a lap in an empty map with only cones. 
Use the following settings.json:

{
  "SettingsVersion": 1.2,
  "Vehicles": {
    "FSCar": {
      "DefaultVehicleState": "",
      "EnableCollisionPassthrogh": false,
      "EnableCollisions": true,
      "AllowAPIAlways": true,
      "RC":{
          "RemoteControlID": -1
      },
      "Sensors": {
        "Gps" : {
          "SensorType": 3,
          "Enabled": true
        },
        "Lidar": {
          "SensorType": 6,
          "Enabled": true,
          "X": 1.3, "Y": 0, "Z": 0.1,
          "Roll": 0, "Pitch": 0, "Yaw" : 0,
          "NumberOfLasers": 1,
          "PointsPerScan": 500,
          "VerticalFOVUpper": 0,
          "VerticalFOVLower": 0,
          "HorizontalFOVStart": -90,
          "HorizontalFOVEnd": 90,
          "RotationsPerSecond": 10,
          "DrawDebugPoints": true
        }
      },
      "Cameras": {},
      "X": 0, "Y": 0, "Z": 0,
      "Pitch": 0, "Roll": 0, "Yaw": 0
    }
  }
}
"""

import sys
import os
import time

import numpy
import math

import cv2

## adds the fsds package located the parent directory to the pyhthon path
sys.path.insert(0, os.path.abspath(os.path.join(os.getenv("HOME"), "Formula-Student-Driverless-Simulator/python")))
import fsds

# connect to the simulator 
client = fsds.FSDSClient()

# Check network connection, exit if not connected
client.confirmConnection()

# After enabling setting trajectory setpoints via the api. 
client.enableApiControl(True)

# Autonomous system constatns
max_throttle = 0.2 # m/s^2
target_speed = 4 # m/s
max_steering = 0.3
cones_range_cutoff = 7 # meters

def pointgroup_to_cone(group):
    average_x = 0
    average_y = 0
    for point in group:
        average_x += point['x']
        average_y += point['y']
    average_x = average_x / len(group)
    average_y = average_y / len(group)
    return {'x': average_x, 'y': average_y}

def distance(x1, y1, x2, y2):
    return math.sqrt(math.pow(abs(x1-x2), 2) + math.pow(abs(y1-y2), 2))


import time

def get_camera():
    global ax1, im1, ax2, im2
    
    #imgL = fsds.string_to_uint8_array(client.simGetImage("cam1", fsds.ImageType.Scene ))


    responses = client.simGetImages([
        fsds.ImageRequest("cam1", fsds.ImageType.Scene, False, False),
        fsds.ImageRequest("cam2", fsds.ImageType.Scene, False, False),
        fsds.ImageRequest("cam3", fsds.ImageType.DepthPlanner, pixels_as_float = True, compress=False)
    ])
    
    response = responses[0]
    # get numpy array
    img1d = numpy.fromstring(response.image_data_uint8, dtype=numpy.uint8) 
    # reshape array to 4 channel image array H X W X 4
    imgL = img1d.reshape(response.height, response.width, 3)

    imgL =cv2.resize(imgL, ( imgL.shape[1]//2, imgL.shape[0]//2 ))
    # original image is fliped vertically
    #imgL = numpy.flipud(img_rgb)

    response = responses[1]
    # get numpy array
    img1d = numpy.fromstring(response.image_data_uint8, dtype=numpy.uint8)
    # reshape array to 4 channel image array H X W X 4
    imgR = img1d.reshape(response.height, response.width, 3)
    imgR =cv2.resize(imgR, ( imgR.shape[1]//2, imgR.shape[0]//2 ))

    # ['__class__', '__delattr__', '__dict__', '__dir__', '__doc__', '__eq__', '__format__', '__ge__', '__getattribute__', '__gt__', '__hash__', '__init__', '__init_subclass__', '__le__', '__lt__', '__module__', '__ne__', '__new__', '__reduce__', '__reduce_ex__', '__repr__', '__setattr__', '__sizeof__', '__str__', '__subclasshook__', '__weakref__', 'camera_name', 'camera_orientation', 'camera_position', 'compress', 'from_msgpack', 'height', 'image_data_float': [0.0], 'image_data_uint8', 'image_type', 'message', 'pixels_as_float': False, 'time_stamp', 'to_msgpack', 'width']


    response = responses[2]
    # get numpy array
    # print(response.image_data_float)
    #print(dir(response))
    #img1d = numpy.fromstring(response.image_data_uint8, dtype=numpy.uint8) 
    
    img1d = numpy.uint8(response.image_data_float)
    # img1d = numpy.array(response.image_data_float)
    # reshape array to 4 channel image array H X W X 4
    imgD = img1d.reshape(response.height, response.width, 1)
    imgD = cv2.resize(imgD, ( imgD.shape[1]//2, imgD.shape[0]//2 ))
    
    imgLg = cv2.cvtColor(imgL, cv2.COLOR_BGR2GRAY)
    imgRg = cv2.cvtColor(imgR, cv2.COLOR_BGR2GRAY)

    stereo = cv2.StereoBM_create(numDisparities=128, blockSize=15)
    # imgD = stereo.compute(imgLg,imgRg)


    cv2.imshow("imgL", imgL)
    cv2.imshow("imgD", imgD)
    
    cv2.waitKey(1)

    print(img1d.shape, imgL.shape)
    pass

def find_cones():
    # Get the pointcloud
    lidardata = client.getLidarData(lidar_name = 'Lidar')

    # no points
    if len(lidardata.point_cloud) < 3:
        return []

    # Convert the list of floats into a list of xyz coordinates
    points = numpy.array(lidardata.point_cloud, dtype=numpy.dtype('f4'))
    points = numpy.reshape(points, (int(points.shape[0]/3), 3))

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

def calculate_steering(cones):
    # If there are more cones on the left, go to the left, else go to the right.
    average_y = 0
    for cone in cones:
        average_y += cone['y']
    average_y = average_y / len(cones)

    if average_y > 0:
        return -max_steering
    else:
        return max_steering

def calculate_throttle():
    gps = client.getGpsData()

    # Calculate the velocity in the vehicle's frame
    velocity = math.sqrt(math.pow(gps.gnss.velocity.x_val, 2) + math.pow(gps.gnss.velocity.y_val, 2))

    # the lower the velocity, the more throttle, up to max_throttle
    return max_throttle * max(1 - velocity / target_speed, 0)

while True:
    cones = find_cones()

    get_camera()

    if len(cones) == 0:
        continue

    car_controls = fsds.CarControls()
    car_controls.steering = calculate_steering(cones)
    car_controls.throttle = calculate_throttle()
    car_controls.brake = 0
    client.setCarControls(car_controls)
