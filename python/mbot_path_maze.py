""" Script to publish waypoints through a maze on an LCM topic """

import lcm
import sys
import numpy as np

sys.path.append("/home/pi/botlab")
from lcmtypes import mbot_motor_command_t, pose_xyt_t, robot_path_t

lc = lcm.LCM("udpm://239.255.76.67:7667?ttl=1")


L = 0.61
# L = 0.30
# x, y, theta, where theta indicates the direction to be pointed when reaching waypoint
waypoints = [
  (0, 0, 0),
  (L, 0, 0),
  (L, -L, -np.pi/2),
  (2*L, -L, 0),
  (2*L, L, np.pi/2),
  (3*L, L, 0),
  (3*L, -L, -np.pi/2),
  (4*L, -L, 0),
  (4*L, 0, np.pi/2),
  (5*L, 0, 0)
]

# drive maze, assuming robot starts at start

path = robot_path_t()

for p in waypoints:
  pose = pose_xyt_t()
  pose.x = p[0]
  pose.y = p[1]
  pose.theta = p[2]
  path.path.append(pose)

path.path_length = len(path.path)


lc.publish("CONTROLLER_PATH", path.encode())
