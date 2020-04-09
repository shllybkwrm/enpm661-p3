# ENPM661 Project 3 Phase 2
# Shelly Bagchi & Omololu Makinde


import math
import time
import numpy as np
from matplotlib.path import Path
from matplotlib.patches import PathPatch
from matplotlib.patches import Ellipse
import matplotlib.pyplot as plt


# Note:  Check these with TAs
w = 10.2
h = 10
w_radius=0.1
sep_dis=1


def convert_RPM_mps (RPM):
    V=(RPM*2*np.pi)/60
    return V



#### IF WE HAVE TIME WE SHOULD CHANGE THIS INTO A GUI AND GET ALL INPUTS ONE TIME
##### Input functions #####
def get_parameters():
##    print("Please enter the rigid robot parameters.")
##    ans=(input("Enter the radius (default=3): "))
##    if ans=='':  radius=3
##    else:  radius=int(ans)
    ans=(input("Enter the obstacle clearance (default=2): "))
    if ans=='':  clearance=0.2
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
    print("\nEnter the initial coordinates of the robot.")
    ans=(input("Enter the x coordinate (default=50): "))
    if ans=='':  x=7
    else:  x=int(ans)
    ans=(input("Enter the y coordinate (default=30): "))
    if ans=='':  y=5
    else:  y=int(ans)
    ans=(input("Enter the starting theta (30-deg increments, default=60): "))
    if ans=='':  theta_s=60
    else:  theta_s=int(ans)

    return [x, y], theta_s

def get_goal():
    print("\nEnter the coordinates of the robot's goal.")
    ans=(input("Enter the target x coordinate (default=7): "))
    if ans=='':  x=7
    else:  x=int(ans)
    ans=(input("Enter the target y coordinate (default=5): "))
    if ans=='':  y=0
    else:  y=int(ans)

    return [x, y]



def drotmatrix(point,angle):
    R=np.array([[np.cos(np.deg2rad(angle)),-(np.sin(np.deg2rad(angle)))],[np.sin(np.deg2rad(angle)),np.cos(np.deg2rad(angle))]])
    b=np.array(point).transpose()
##    b=np.array(point)
    print(np.dot(R,b))
##    print(np.dot(b,R))
    return np.dot(R,b)
# Get input parameters
clearance, RPM1,RPM2 = get_parameters()  #Changed from proj 3-2 
start_point, theta_s = get_start()
goal_point = get_goal()#Changed from proj 3-2 
print()
########### SET ROBOT COORDINATE ##############
robot_x_coord=start_point[0]
robot_y_coord=start_point[1]
goal_x_coord=goal_point[0]
goal_y_coord=goal_point[1]
robot_breadth=2*w_radius
robot_height= sep_dis

def get_points(x_coord,y_coord):
    x=np.linspace((robot_x_coord-robot_breadth/2),(robot_x_coord+robot_breadth/2),robot_breadth+1)
    y=np.linspace((robot_y_coord+robot_height/2),(robot_y_coord-robot_height/2),robot_height+1)
    x_1,y_1=np.meshgrid(x,y, indexing='xy')
    return np.array(list(zip(x_1.flatten(),y_1.flatten())))
robot_points = get_points(robot_x_coord,robot_y_coord)
goal_points = get_points(goal_x_coord,goal_y_coord)
print(robot_points==goal_points)


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
####### CHECKING TO SEE IF ROBOT IS IN OBSTACLE ################
def inside_obstacle(points):
    effective_clearance = clearance
    inside_polygons = (path.contains_points(points, radius=effective_clearance))#true if it is inside the polygon,otherwise false
    inside_circle1= (circle1.contains_points(points,radius=effective_clearance))
    inside_circle2= (circle2.contains_points(points,radius=effective_clearance))
    inside_circle3= (circle3.contains_points(points,radius=effective_clearance))
    inside_circle4= (circle4.contains_points(points,radius=effective_clearance))
    return (all(inside_polygons==True) or all(inside_circle1==True) or all(inside_circle2==True) or all(inside_circle3==True) or all(inside_circle4==True) )

if inside_obstacle(goal_points):
    print("error:  goal is inside obstacle!")
    exit()
elif inside_obstacle(robot_points):
    print("error:  robot starts inside obstacle!")
    exit()
###########


####################### A STAR ################
class Node:
    def __init__(self, node_no, coord, parent=None, g=0, h=0, f=0, theta=0):
        self.node_no = node_no
        self.parent = parent
        self.coord = coord
        self.g=g
        self.h=h
        self.f=f
        self.theta = theta

degree_list=np.linspace(0, 360, 12, endpoint=False, dtype=int)
############ Map for duplicate checking
visited_matrix = np.zeros((int(w*2),h*2,12), dtype=bool)
i = np.where(degree_list==theta_s)
visited_matrix[start_point[0], start_point[1], i] = True
#########
def distance_2(p1,p2):
    distance = math.sqrt( ((p1[0]-p2[0])**2)+((p1[1]-p2[1])**2) )
    return distance
#########
def generate_node_successor(coord):
    actions=[[0,RPM1],[RPM1,0],[RPM1,RPM1],[0,RPM2],[RPM2,0],[RPM2,RPM2],[RPM1,RPM2],[RPM2,RPM1]]
    for action in actions:
        new_positions=[]
        thetas=[]
        t=0
        r=int(w_radius)
        L=sep_dis
        dt=0.1
        X1=0
        Y1=0
        dtheta=0
        for i,angle in enumerate(degree_list):
            Theta0=np.deg2rad(angle)
            Theta1=Theta0
            while t<1:
                t=t+dt
                coord[0]=coord[0]+X1
                coord[1]=coord[1]+Y1
                dx=r*(action[0]+action[1])*math.cos(Theta1)*dt
                dy=r*(action[0]+action[1])*math.sin(Theta1)*dt
                dtheta=(r/L)*(action[1]-action[0])*dt
                X1=X1+dx
                Y1=Y1+dy
                Theta1=Theta1+0.5*dtheta
    ##            plt.quiver(coord[0], coord[1], X1, Y1,units='xy' ,scale=1,color= 'r',width =0.2, headwidth = 1,headlength=0)
                Xn=coord[0]+X1
                Yn=coord[1]+Y1
                Thetan=180*(Theta1)/3.14
                new_point=np.array([Xn,Yn])
            return new_point,Thetan
            new_point = ( np.round(new_point*2, decimals=0) ) / 2
            # Check for duplicates
            a = int(new_point[0]*2)
            b = int(new_point[1]*2)
            #deg = int(np.rad2deg(t))
            if a<0 or a>w*2:
                continue
            if b<0 or b>h*2:
                continue
            if inside_obstacle([new_point]):
                continue
            if visited_matrix[a, b, i]:
                #print("node already visited: ", new_point, angle)
                continue
            #else:
            visited_matrix[a, b, i] = True
            new_positions.append(new_point)
            thetas.append(angle)
            return new_positions, thetas
    

pos,angle=generate_node_successor(start_point)
print("position ", pos,"angle ", angle)

    


def get_gscore(previous,current):
    return (previous.g + distance_2(previous.coord, current))
    # Calculate g incrementally, so angle of approach is not needed

def get_hscore(current):
    return distance_2(current, goal_point)



# plot FROM parent to node at node angle (angle of arrival)
def plot_vector(node, c='k', w=0.025):
    if node.parent==None:  return
    x=node.parent.coord[0]
    y=node.parent.coord[1]
    rad = np.deg2rad(node.theta)
    q = step*np.cos(rad)
    v = step*np.sin(rad)
    # Plot vector
    ax.quiver(x, y, q, v, units='xy', angles='xy', scale=1, color=c, width=w)
  




def graph_search(start_point,goal_point):
    start_node = Node(0, start_point, g=0, h=starth, f=0+starth, theta=theta_s) 
    node_q = [start_node]  # put the startNode on the openList with f=0
    explored_nodes = [] # points visited
    #child_nodes = []  # closed list
    ##final_nodes.append(node_q[0])  # Only writing data of nodes in seen
    ##visited_coord.append(node_q[0].coord)
    node_counter = 0  # To define a unique ID to all the nodes formed

    ##for i in range(1):#while node_q:  # UNCOMMENT FOR DEBUGGING 
    while node_q: #while the OPEN list is not empty
        current_root = node_q[0]##############################change current root to equal node with smallest f value#############
        current_index = 0
        for index,thing in enumerate(node_q):#let the currentNode equal the node with the lowest cost
            if thing.f < current_root.f:
                current_root = thing
                current_index = index
        node_q.pop(current_index)
        explored_nodes.append(current_root)
        plot_vector(current_root)
        print("current node: ", current_root.coord, current_root.theta, current_root.f)
        if current_root.coord[0]==goal_point[0] and current_root.coord[1]==goal_point[1]:# and current_root.theta==theta_g:
            print("\nGoal reached:  ", current_root.coord, current_root.theta, current_root.f)
            return current_root

        child_coords,thetas = generate_node_successor(current_root.coord)
        # Having issues when no children found so check that here
        if child_coords.size==0 or thetas.size==0:
            continue    
        for child_point,theta in zip(child_coords, thetas):
            #print("child_point: ", child_point, theta)
            node_counter+=1
            #print("node count: ", node_counter)
            tempg=get_gscore(current_root,child_point)
            temph=get_hscore(child_point)
            child_node = Node(node_counter, child_point, parent=current_root, g=tempg, h=temph, f=tempg+temph, theta=theta)
            # NOTE:  Add plotting function here

            #child_nodes.append(child_node)
        # Redundant line, removing for efficiency
        #for child in child_nodes:
            # Adjusted this to replace explored nodes if the node is found again with lower cost ###
            for i,explored in enumerate(explored_nodes):
                if child_node.coord[1]==explored.coord[0] and child_node.coord[1]==explored.coord[1] and child_node.g<explored.g:
                    print("Reached previously explored node with lower cost, replacing")
                    explored_nodes[i] = child_node
                    #continue
            for item in node_q:
                if (child_node.coord.tolist()==item.coord.tolist()) and child_node.g>item.g:
                    print("Coordinates present with lower cost, not adding to queue")
                    continue
            node_q.append(child_node)

        print("node count: ", node_counter)


def find_path(node):  # To find the path from the goal node to the starting node
    p = []
    p.append(node)
    parent_node = node.parent
    while parent_node is not None:
        p.append(parent_node.coord)
        parent_node = parent_node.parent
        plot_vector(parent_node, 'g', w=0.2)

    return list(reversed(p))



print("\nThe start is", start_point, theta_s)
print("The goal is", goal_point)
starth = get_hscore(start_point)
print("The distance to goal is", starth, '\n')

start_time = time.time()

goal_node=graph_search(start_point,goal_point)
result=find_path(goal_node)
print(result)

end_time = time.time()
print("Total execution time:", end_time-start_time)
    
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
