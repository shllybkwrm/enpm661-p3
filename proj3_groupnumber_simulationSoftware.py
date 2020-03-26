# ENPM661 Project 3 Phase 2
# Shelly Bagchi & Omololu Makinde


import math
import time
import numpy as np
from matplotlib.path import Path
from matplotlib.patches import PathPatch
from matplotlib.patches import Ellipse
import matplotlib.pyplot as plt


w = 300
h = 200
r=0.1
L=1

def convert_RPM_mps (RPM):
    V=(RPM*2*np.pi)/60
    return V

####IF WE HAVE TIME WE SHOULD CHANGE THIS INTO A GUI AND GET ALL INPUTS ONE TIME
##### Input functions #####
def get_parameters():
##    print("Please enter the rigid robot parameters.")
##    ans=(input("Enter the radius (default=3): "))
##    if ans=='':  radius=3
##    else:  radius=int(ans)
    ans=(input("Enter the obstacle clearance (default=2): "))
    if ans=='':  clearance=2
    else:  clearance=int(ans)
##    ans=(input("Enter the robot step size (1-10, default=1): "))
##    if ans=='' or int(ans)<1:  step=1
##    elif int(ans)>10:  step=10
##    else:  step=int(ans)
    ans=(input("Enter the left wheel speed in RPM (default=5): "))
    if ans=='':  RPM1=5
    else:  RPM1=int(ans)
    ans=(input("Enter the right wheel speed in RPM (default=5): "))
    if ans=='':  RPM2=5
    else:  RPM2=int(ans) 

##    return radius, clearance, step, RPM1, RPM2
    return clearance, RPM1, RPM2

def get_start():
    print("\nPlease enter the initial coordinates of the robot.")
    ans=(input("Enter the x coordinate (default=50): "))
    if ans=='':  x=0
    else:  x=int(ans)
    ans=(input("Enter the y coordinate (default=30): "))
    if ans=='':  y=0
    else:  y=int(ans)
    ans=(input("Enter the starting theta (30-deg increments, default=60): "))
    if ans=='':  theta_s=45
    else:  theta_s=int(ans)

    return [x, y], theta_s

def get_goal():
    print("\nPlease enter the coordinates of the robot's goal.")
    ans=(input("Enter the target x coordinate (default=150): "))
    if ans=='':  x=5
    else:  x=int(ans)
    ans=(input("Enter the target y coordinate (default=150): "))
    if ans=='':  y=5
##    ans=(input("Enter the goal theta (30-deg increments, default=same as start): "))
##    if ans=='':  theta_g=theta_s
##    else:  theta_g=int(ans)

##    return [x, y], theta_g
    return [x, y]


# Get input parameters
clearance, u_l,u_r = get_parameters()  #Changed from proj 3-2 
start_point, theta_s = get_start()
goal_point = get_goal()#Changed from proj 3-2 
print()
