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

###
# Multi Threading
# Keeping track of the list of threads that we run :
# {
#   "GRAPH": graph_thread,
#   "PYGAME": pygame_thread
# }
###
import threading
global THREADS
THREADS = dict()

global RUNNING
RUNNING = True

"""
Start of 3D plotting code
"""

import numpy as np
import os


###
# Plotter Code
###
#
from pyqtgraph.Qt import QtCore, QtGui
import pyqtgraph.opengl as gl
import numpy as np
import sys

import math

global POINTS
POINTS = [[0,0,0],]


global min_height, max_height, colors, calculated, max_dist, min_dist, dist_range
calculated = False


def update_graph():
    global graph_region, POINTS
    global min_height, max_height, colors, calculated, max_dist, min_dist, dist_range

    POINTS = numpy.array(POINTS)

    if not calculated:
        colors = np.ones(shape=(len(POINTS), 4), dtype=np.float32)
        min_height = min(POINTS[:, 2])
        max_height = max(POINTS[:, 2])
        for i in range(len(POINTS)):
            if POINTS[i][2] - min_height < 0.2 * (max_height - min_height):
                colors[i] = (0.3, 0.3, 0.3, 0.3)

        max_dist = 0
        min_dist = 0
        for x, y, z in POINTS:
            d = math.sqrt(x**2 + y**2 + z**2)
            if d<min_dist:
                min_dist = d
            if d>max_dist:
                max_dist = d

        dist_range = max_dist + min_dist
        # 1.0314498 79.98652 78.95507
        # 0 67.95823145281865 67.95823145281865
        print(min_dist, max_dist, dist_range)
        calculated = True

    colors = np.ones(shape=(len(POINTS), 4), dtype=np.float32)

    if len(POINTS)>0:
        a = np.array(POINTS)
        a[:, 2] = a[:, 2] - min_height
        # a[:, 1] = a[:, 1] * -1
        a[:, 0] = a[:, 0] - 10
        #a = np.array([(1,1,1), (2,2,2), (3,3,3)])
        graph_region.setData(pos=a, color=colors)
        # graph_region.setData(pos=np.flip(np.array(POINTS), 2), color=colors)


def start_graph():
    print("Setting up graph")
    global app, graph_region, w, g, d3, t
    app = QtGui.QApplication([])
    w = gl.GLViewWidget()
    w.resize(800, 600)
    w.opts['distance'] = 20
    w.show()
    w.setWindowTitle('LIDAR Point Cloud')

    g = gl.GLGridItem()
    #g.translate(0,0,-10)
    w.addItem(g)

    #pos3 = np.zeros((100, 100, 3))
    #pos3[:, :, :2] = np.mgrid[:100, :100].transpose(1, 2, 0) * [-0.1, 0.1]
    #pos3 = pos3.reshape(10000, 3)
    #d3 = (pos3 ** 2).sum(axis=1) ** 0.5

    graph_region = gl.GLScatterPlotItem(pos=np.zeros((1, 3), dtype=np.float32), color=(0, 1, 0, 0.5), size=0.1, pxMode=False)
    #graph_region.rotate(180, 1, 0, 0)
    #graph_region.translate(0, 0, 2.4)
    w.addItem(graph_region)
    t = QtCore.QTimer()
    t.timeout.connect(update_graph)
    t.start(50)

    QtGui.QApplication.instance().exec_()
    global RUNNING
    RUNNING = False
    print("\n[STOP]\tGraph Window closed. Stopping...")

def process_lidar(data):
    #try:
    global POINTS
    #POINTS = data.reshape(-1, 4).tolist()
    #POINTS = np.delete(POINTS, 3, 1)
    POINTS = data
    #for i in range(0, 20):print(i, " : ", POINTS[i])
    #except Error as e:
    #    print("Error")
    #    pass


import sys
import os
import time

import numpy
import math
#import matplotlib.pyplot as plt

#import cv2

## adds the fsds package located the parent directory to the pyhthon path

# sys.path.insert(0, os.path.abspath(os.path.join(os.path.dirname(__file__), '..')))
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

"""
#create two subplots
global ax1, im1, ax2, im2, ax3, im3
ax1 = plt.subplot(1,3,1)
ax2 = plt.subplot(1,3,2)
ax3 = plt.subplot(1,3,3)

img_width = 512
img_height = 1382
im1 = ax1.imshow(numpy.zeros((img_width, img_height, 3), dtype=numpy.uint8))
im2 = ax2.imshow(numpy.zeros((img_width, img_height, 3), dtype=numpy.uint8))
im3 = ax3.imshow(numpy.zeros((img_width, img_height, 3), dtype=numpy.uint8))
"""

#import cv2

def get_camera():
    
    responses = client.simGetImages([
        fsds.ImageRequest("cam1", fsds.ImageType.Scene, False, False),
        fsds.ImageRequest("cam2", fsds.ImageType.Scene, False, False),
        fsds.ImageRequest("cam1", fsds.ImageType.DisparityNormalized, False, False)
    ])
    
    response = responses[0]
    # get numpy array
    img1d = numpy.fromstring(response.image_data_uint8, dtype=numpy.uint8) 
    # reshape array to 4 channel image array H X W X 4
    imgL = img1d.reshape(response.height, response.width, 3)
    # original image is fliped vertically
    #imgL = numpy.flipud(img_rgb)

    response = responses[1]
    # get numpy array
    img1d = numpy.fromstring(response.image_data_uint8, dtype=numpy.uint8) 
    # reshape array to 4 channel image array H X W X 4
    imgR = img1d.reshape(response.height, response.width, 3)

    response = responses[2]
    # get numpy array
    img1d = numpy.fromstring(response.image_data_uint8, dtype=numpy.uint8) 
    # reshape array to 4 channel image array H X W X 4
    imgD = img1d.reshape(response.height, response.width, 3)
    

    print(img1d.shape, imgL.shape)

    #cv2.imshow("imgL", imgL)
    #im1.set_data(imgL)
    #im2.set_data(imgR)
    #im3.set_data(imgD)

        
    #if (im2 == False): im2 = ax2.imshow(grab_frame(cap2))
    #cv2.imshow("Left", imgL)
    #cv2.imshow("Right", )
    #plt.show()
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


def main():
    while True:
        # 20 hz
        #plt.pause(0.05)

        #plt.clf()
        #plt.axis([-cones_range_cutoff, cones_range_cutoff, -2, cones_range_cutoff])
        
        cones = find_cones()

        get_camera()

        if len(cones) == 0:
            continue

        car_controls = fsds.CarControls()
        car_controls.steering = calculate_steering(cones)
        car_controls.throttle = calculate_throttle()
        car_controls.brake = 0
        client.setCarControls(car_controls)

        # draw cones
        #for cone in cones: plt.scatter(x=-1*cone['y'], y=cone['x'])
    
main_thread = threading.Thread(target=main, args=())
#file_reader_thread.daemon = True  # Daemonize thread
# thread.start()  # Start the execution this is done after we have connected to the simulator
THREADS["MAIN"] = main_thread


#plt.show()

print("\nStarting Threads : ")
for t in THREADS:
    print("[START]\t", t)
    THREADS[t].start()


if __name__ == '__main__':
    if (sys.flags.interactive != 1) or not hasattr(QtCore, 'PYQT_VERSION'):
        start_graph()
