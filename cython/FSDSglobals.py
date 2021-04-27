import numpy as np
import math

state_grid_size = 96#20

action_size = 20
state_size = state_grid_size**2

# qtable = np.zeros((state_size, action_size))

total_episodes = 15000        # Total episodes
learning_rate = 0.8           # Learning rate
max_steps = 99                # Max steps per episode
gamma = 0.95                  # Discounting rate

# Exploration parameters
# epsilon = 1.0                 # Exploration rate
max_epsilon = 1.0             # Exploration probability at start
min_epsilon = 0.01            # Minimum exploration probability 
decay_rate = 0.005             # Exponential decay rate for exploration prob

MISSIONS = ("Autocross", "Trackdrive")
TRACKS = ("TrainingMap", "CompetitionMapTestday1", "CompetitionMapTestday2", "CompetitionMapTestday3", "CompetitionMap1", "CompetitionMap2", "CompetitionMap")

# Autonomous system constatns
max_throttle = 0.2 # m/s^2
target_speed = 4 # m/s
max_steering = 0.3
cones_range_cutoff = state_grid_size // 2# 10 # meters

OPTIMIZE_FPS = False
SCALE_FACTOR = 4.5
CONE_RADIUS = 1
CAR_RADIUS = 2

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