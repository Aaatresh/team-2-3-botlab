import pygame
from pygame.locals import *
import time
import numpy as np
import sys
#sys.path.append("/home/pi/botlab/lcmtypes.")
sys.path.append("../../")
import lcm
from lcmtypes import mbot_motor_command_t

LIN_VEL_CMD = 1.5
ANG_VEL_CMD = 6.28

lc = lcm.LCM("udpm://239.255.76.67:7667?ttl=1")
pygame.init()
pygame.display.set_caption("MBot TeleOp")
time.sleep(0.5)
while True:
    fwd_vel = 0.0
    turn_vel = 0.0
    for event in pygame.event.get():
        if event.type==pygame.QUIT:
            pygame.quit()
            sys.exit()
    key_input = pygame.key.get_pressed()  
    if key_input[pygame.K_LEFT]:
        turn_vel += 1.0
    if key_input[pygame.K_UP]:
        fwd_vel +=1.0
    if key_input[pygame.K_RIGHT]:
        turn_vel -= 1.0
    if key_input[pygame.K_DOWN]:
        fwd_vel -= 1.0
    command = mbot_motor_command_t()
    command.trans_v = fwd_vel * LIN_VEL_CMD
    command.angular_v = turn_vel * ANG_VEL_CMD
    lc.publish("MBOT_MOTOR_COMMAND",command.encode())
    print("published")
    time.sleep(0.05)

