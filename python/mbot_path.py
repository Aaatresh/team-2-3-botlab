
import time
import lcm
import sys
import numpy as np

sys.path.append("/home/pi/botlab")
from lcmtypes import mbot_motor_command_t, pose_xyt_t, robot_path_t

LIN_VEL_CMD = 1.5
ANG_VEL_CMD = 6.28

lc = lcm.LCM("udpm://239.255.76.67:7667?ttl=1")


# drive square (assumes i start at (0,0,0)

path = robot_path_t()

pose = pose_xyt_t()

#first corner 
pose = pose_xyt_t()
pose.x = 1.0;
pose.y = 0.0;
pose.theta = 0.0
path.path.append(pose)

# turn
pose = pose_xyt_t()
pose.x = 1.0;
pose.y = 0.0;
pose.theta = np.pi/2;


path.path.append(pose)

# second corner
pose = pose_xyt_t()

pose.x = 1.0;
pose.y = 1.0;
pose.theta = np.pi/2
path.path.append(pose)

# turn
pose = pose_xyt_t()
pose.x = 1.0;
pose.y = 1.0;
pose.theta = np.pi;
path.path.append(pose)


#third corner 
pose = pose_xyt_t()
pose.x = 0.0;
pose.y = 1.0;
pose.theta = np.pi

path.path.append(pose)


# turn
pose = pose_xyt_t()
pose.x = 1.0;
pose.y = 0.0;
pose.theta = 1.5*np.pi;

path.path.append(pose)


#fourth corner 
pose = pose_xyt_t()
pose.x = 0.0;
pose.y = 0.0;
pose.theta = 1.5*np.pi;

path.path.append(pose)


# turn
pose = pose_xyt_t()
pose.x = 0.0;
pose.y = 0.0;
pose.theta = 2.0*np.pi;

path.path.append(pose)

path.path_length = len(path.path)
lc.publish("CONTROLLER_PATH", path.encode())
