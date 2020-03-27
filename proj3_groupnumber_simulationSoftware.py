# ENPM661 Project 3 Phase 2
# Shelly Bagchi & Omololu Makinde


import math
import time
import numpy as np
from matplotlib.path import Path
from matplotlib.patches import PathPatch
from matplotlib.patches import Ellipse
import matplotlib.pyplot as plt


w = 10.2
h = 10
w_radius=0.1
sep_dis=1

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
    if ans=='':  x=5
    else:  x=int(ans)
    ans=(input("Enter the y coordinate (default=30): "))
    if ans=='':  y=3
    else:  y=int(ans)
    ans=(input("Enter the starting theta (30-deg increments, default=60): "))
    if ans=='':  theta_s=45
    else:  theta_s=int(ans)

    return [x, y], theta_s

def get_goal():
    print("\nPlease enter the coordinates of the robot's goal.")
    ans=(input("Enter the target x coordinate (default=150): "))
    if ans=='':  x=8
    else:  x=int(ans)
    ans=(input("Enter the target y coordinate (default=150): "))
    if ans=='':  y=9
    return [x, y]

def drotmatrix(point,angle):
    R=np.array([[np.cos(np.deg2rad(angle)),-(np.sin(np.deg2rad(angle)))],[np.sin(np.deg2rad(angle)),np.cos(np.deg2rad(angle))]])
    b=np.array(point).transpose()
##    b=np.array(point)
    print(np.dot(R,b))
##    print(np.dot(b,R))
    return np.dot(R,b)
# Get input parameters
clearance, u_l,u_r = get_parameters()  #Changed from proj 3-2 
start_point, theta_s = get_start()
goal_point = get_goal()#Changed from proj 3-2 
print()
########### SET ROBOT COORDINATE ##############
robot_x_coord=start_point[0]
robot_y_coord=start_point[1]
goal_x_coord=goal_point[0]
goal_y_coord=goal_point[1]
robot_breadth=2*w_radius 
robot_height=sep_dis
x=np.linspace((robot_x_coord-robot_breadth/2),(robot_x_coord+robot_breadth/2),robot_breadth+1,dtype=int)
y=np.linspace((robot_y_coord+robot_height/2),(robot_y_coord-robot_height/2),robot_height+1,dtype=int)
x_1,y_1=np.meshgrid(x,y, indexing='xy')
points = np.array(list(zip(x_1.flatten(),y_1.flatten())))
############# PLOTTING THE ROBOT - change to circle? ################
rcodes = [Path.MOVETO] + [Path.LINETO]*3 + [Path.CLOSEPOLY]
rvertices = [((robot_x_coord-robot_breadth/2), (robot_y_coord-robot_height/2)), ((robot_x_coord-robot_breadth/2), (robot_y_coord+robot_height/2)), ((robot_x_coord+robot_breadth/2), (robot_y_coord+robot_height/2)), ((robot_x_coord+robot_breadth/2), (robot_y_coord-robot_height/2)), (0, 0)]
robot = Path(rvertices,rcodes)
robotpatch = PathPatch(robot, facecolor='green', edgecolor='green')
print("vertices ",rvertices,"shape ", np.shape(rvertices))
rotvertices=[]
for i in np.array(rvertices):
    i=np.array(i)
    t=i-rvertices[0]
    print("t",t)
    roti=drotmatrix(t,theta_s)
    rotvertices.append((roti+rvertices[0]))
##print("new vertices ",rotvertices,"shape ", np.shape(rotvertices))
rotcodes = [Path.MOVETO] + [Path.LINETO]*3 + [Path.CLOSEPOLY]
rotrobot = Path(rotvertices,rotcodes)
rotrobotpatch = PathPatch(rotrobot, facecolor='blue', edgecolor='blue')
########### PLOTTING THE GOAL - change to circle? ################
gcodes = [Path.MOVETO] + [Path.LINETO]*3 + [Path.CLOSEPOLY]
gvertices = [((goal_x_coord-robot_breadth/2), (goal_y_coord-robot_height/2)), ((goal_x_coord-robot_breadth/2), (goal_y_coord+robot_height/2)), ((goal_x_coord+robot_breadth/2), (goal_y_coord+robot_height/2)), ((goal_x_coord+robot_breadth/2), (goal_y_coord-robot_height/2)), (0, 0)]
goal = Path(gvertices,gcodes)
goalpatch = PathPatch(goal, facecolor='red', edgecolor='red')
######### CIRCLE OBSTACLES #####################
circle1=Path.circle((5,5),radius=1,readonly=False)
circle2=Path.circle((3,2),radius=1,readonly=False)
circle3=Path.circle((7.2,2),radius=1,readonly=False)
circle4=Path.circle((7.2,8),radius=1,readonly=False)
pathpatch1 = PathPatch(circle1, facecolor='None', edgecolor='blue')
pathpatch2 = PathPatch(circle2, facecolor='None', edgecolor='blue')
pathpatch3 = PathPatch(circle3, facecolor='None', edgecolor='blue')
pathpatch4 = PathPatch(circle4, facecolor='None', edgecolor='blue')
############# RECTANGULAR OBSTACLES #############
vertices = []
codes = []
########## POLYGON OBSTACLES ###################
codes = [Path.MOVETO] + [Path.LINETO]*3 + [Path.CLOSEPOLY]
vertices = [(0.25,5.25), (1.75,5.25), (1.75,3.75), (0.25,3.75),(0, 0)]
codes += [Path.MOVETO] + [Path.LINETO]*3 + [Path.CLOSEPOLY]
vertices += [(2.25,8.75), (3.75,8.75), (3.75,7.25), (2.25,7.25), (0, 0)]
codes += [Path.MOVETO] + [Path.LINETO]*3 + [Path.CLOSEPOLY]
vertices += [(8.45,5.25), (9.95,5.25), (9.95,3.75), (8.45,3.75), (0, 0)]
vertices = np.array(vertices, float)
path = Path(vertices, codes)
pathpatch = PathPatch(path, facecolor='None', edgecolor='blue')
######## PLOTTING #####################
fig, ax = plt.subplots()
ax.add_patch(rotrobotpatch)
ax.add_patch(goalpatch)
ax.add_patch(pathpatch)
ax.add_patch(pathpatch1)
ax.add_patch(pathpatch2)
ax.add_patch(pathpatch3)
ax.add_patch(pathpatch4)
##ax.add_patch(pathpatch3)
ax.set_title('Map Space')
ax.autoscale_view()
plt.xlim(0,w)
plt.ylim(0,h)
plt.show()
