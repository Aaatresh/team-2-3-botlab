""" Script to publish waypoints on an LCM topic """

import lcm
import sys
import numpy as np

sys.path.append("/home/pi/botlab")
from lcmtypes import pose_xyt_t, robot_path_t

lc = lcm.LCM("udpm://239.255.76.67:7667?ttl=1")

L = 1.5

# x, y, theta, where theta indicates the direction to be pointed when reaching waypoint
waypoints = [
  (0,0,0),
  (L, 0, 0),
  (L, L, np.pi),
  #(L/2, L/2, np.pi/2), # if you want to make it a M instead of a square
  #(0, L, np.pi),
  #(0, 0, 1.5*np.pi)
]

# drive square, assuming robot starts at start

path = robot_path_t()

for p in waypoints:
  pose = pose_xyt_t()
  pose.x = p[0]
  pose.y = p[1]
  pose.theta = p[2]
  path.path.append(pose)

path.path_length = len(path.path)


lc.publish("CONTROLLER_PATH", path.encode())