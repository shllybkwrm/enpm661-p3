# ENPM661 Project 3 Phase 2
# Shelly Bagchi & Omololu Makinde

import math
import time
import numpy as np
from matplotlib.path import Path
from matplotlib.patches import PathPatch
from matplotlib.patches import Ellipse
import matplotlib.pyplot as plt


# Note:  Values now in mm - original map given in meters
# The outer square is 10,200mm but I only used the inner
w = 10000
h = 10000
rob_radius=354//2
w_radius=76//2
w_circ = np.pi*76
# TB2 max speed:  .65 m/s

def convert_RPM_mmps (RPM):
    return ( (RPM/60)*w_circ )
    

####IF WE HAVE TIME WE SHOULD CHANGE THIS INTO A GUI AND GET ALL INPUTS ONE TIME
##### Input functions #####
def get_parameters():
##    print("Please enter the rigid robot parameters.")
##    ans=(input("Enter the radius (default=3): "))
##    if ans=='':  radius=3
##    else:  radius=int(ans)
    ans=(input("Enter the obstacle clearance (default=50mm): "))
    if ans=='':  clearance=50
    else:  clearance=int(ans)
##    ans=(input("Enter the robot step size (1-10, default=1): "))
##    if ans=='' or int(ans)<1:  step=1
##    elif int(ans)>10:  step=10
##    else:  step=int(ans)
    ans=(input("Enter the left wheel speed in RPM (default=5): "))
    if ans=='':  RPM_L=5
    else:  RPM_L=int(ans)
    ans=(input("Enter the right wheel speed in RPM (default=5): "))
    if ans=='':  RPM_R=5
    else:  RPM_R=int(ans) 

##    return radius, clearance, step, RPM1, RPM2
    return clearance, RPM_L, RPM_R

def get_start():
    print("\nPlease enter the initial coordinates of the robot.")
    ans=(input("Enter the x coordinate (default=1000mm): "))
    if ans=='':  x=1000
    else:  x=int(ans)
    ans=(input("Enter the y coordinate (default=1000mm): "))
    if ans=='':  y=1000
    
    else:  y=int(ans)
    ans=(input("Enter the starting theta (30-deg increments, default=60): "))
    if ans=='':  theta_s=60
    else:  theta_s=int(ans)

    return [x, y], theta_s

def get_goal():
    print("\nPlease enter the coordinates of the robot's goal.")
    ans=(input("Enter the target x coordinate (default=3000mm): "))
    if ans=='':  x=3000
    else:  x=int(ans)
    ans=(input("Enter the target y coordinate (default=2000mm): "))
    if ans=='':  y=2000
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
clearance, RPM_L, RPM_R = get_parameters()  #Changed from proj 3-2 
start_point, theta_s = get_start()
goal_point = get_goal()  #Changed from proj 3-2 
print()
########### SET ROBOT COORDINATE ##############
### Adjusted to be in matplotlib coords
robot_x_coord=start_point[0]+w//2
robot_y_coord=start_point[1]+h//2
goal_x_coord=goal_point[0]+w//2
goal_y_coord=goal_point[1]+h//2

robot_breadth=2*rob_radius 
robot_height= 2*rob_radius

def get_points(x_coord,y_coord):
    x=np.linspace((robot_x_coord-robot_breadth//2), (robot_x_coord+robot_breadth//2), robot_breadth+1, dtype=int)
    y=np.linspace((robot_y_coord+ robot_height//2), (robot_y_coord- robot_height//2),  robot_height+1, dtype=int)
    x_1,y_1=np.meshgrid(x,y, indexing='xy')
    return np.array(list(zip(x_1.flatten(),y_1.flatten())))
robot_points = get_points(robot_x_coord,robot_y_coord)
goal_points  = get_points( goal_x_coord, goal_y_coord)
#print(robot_points==goal_points)


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
circle1=Path.circle((5000,5000),radius=1000,readonly=False)
circle2=Path.circle((3000,2000),radius=1000,readonly=False)
circle3=Path.circle((7200,2000),radius=1000,readonly=False)
circle4=Path.circle((7200,8000),radius=1000,readonly=False)
pathpatch1 = PathPatch(circle1, facecolor='None', edgecolor='blue')
pathpatch2 = PathPatch(circle2, facecolor='None', edgecolor='blue')
pathpatch3 = PathPatch(circle3, facecolor='None', edgecolor='blue')
pathpatch4 = PathPatch(circle4, facecolor='None', edgecolor='blue')
############# RECTANGULAR OBSTACLES #############
vertices = []
codes = []
########## POLYGON OBSTACLES ###################
codes = [Path.MOVETO] + [Path.LINETO]*3 + [Path.CLOSEPOLY]
vertices = [(250,5250), (1750,5250), (1750,3750), (250,3750),(0, 0)]
codes += [Path.MOVETO] + [Path.LINETO]*3 + [Path.CLOSEPOLY]
vertices += [(2250,8750), (3750,8750), (3750,7250), (2250,7250), (0, 0)]
codes += [Path.MOVETO] + [Path.LINETO]*3 + [Path.CLOSEPOLY]
vertices += [(8450,5250), (9950,5250), (9950,3750), (8450,3750), (0, 0)]
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
    print(">> Error:  goal is inside obstacle!")
    exit()
elif inside_obstacle(robot_points):
    print(">> Error:  robot starts inside obstacle!")
    exit()
###########


########## PLOTTING #####################
fig, ax = plt.subplots()
plt.axis('square')
plt.xlim(0,w)
plt.ylim(0,h)
#ax.autoscale_view()
ax.add_patch(rotrobotpatch)
ax.add_patch(goalpatch)
ax.add_patch(pathpatch)
ax.add_patch(pathpatch1)
ax.add_patch(pathpatch2)
ax.add_patch(pathpatch3)
ax.add_patch(pathpatch4)
ax.set_title('Map Space')
#plt.show()


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


def distance_2(p1,p2):
    distance = math.sqrt( ((p1[0]-p2[0])**2)+((p1[1]-p2[1])**2) )
    return distance


### Map for duplicate checking
# Dicretize space to 100mm; action space is now 8 not 12
spacing=100
w_dis = w//spacing
h_dis = h//spacing
visited_matrix = np.zeros((w_dis,h_dis,8), dtype=bool)

# Might need to change this for new action set
degree_list=np.linspace(0, 360, 12, endpoint=False, dtype=int)
i = np.where(degree_list==theta_s)
visited_matrix[start_point[0]//spacing, start_point[1]//spacing, i] = True


def generate_node_successor(coord,thetaIn,action,action_id):    
    new_positions=[]
    thetas=[]
    t=0
    r=w_radius
    L=rob_radius
    dt=0.1
    X0=coord[0]
    Y0=coord[1]
    X1=0
    Y1=0
    dtheta=0
    ThetaRad=np.deg2rad(thetaIn)

    while t<1:
        t=t+dt
        X0+=X1
        Y0+=Y1

        # Note:  These should be velocities, not RPM!
        # Converted them in parent A* function
        dx=r*(action[0]+action[1])*math.cos(ThetaRad)*dt
        dy=r*(action[0]+action[1])*math.sin(ThetaRad)*dt
        dtheta=(r/L)*(action[1]-action[0])*dt

        X1+=dx
        Y1+=dy
        ThetaRad+=0.5*dtheta

        # Plot here to get a curve rather than vector
        ax.quiver(X0, Y0, X1, Y1, units='xy', scale=1, color='k', width=0.2, headwidth=1, headlength=0)
    
    Xn=X0+X1
    Yn=X0+Y1
    ThetaDeg=np.rad2deg(ThetaRad)
    print("final child coords & angle: ", Xn,Yn,ThetaDeg, "from action", action_id)
    new_point=np.array([Xn,Yn])

    ### Check bounds accounting for origin at center
    if Xn<-w/2 or Xn>w/2:  
        return [],[]
    if Yn<-h/2 or Yn>h/2:
        return [],[]
    # Check collisions
    if inside_obstacle([new_point]): 
        return [],[]
    
    ### Check for duplicates
    # Discretize to nearest 100mm
    new_point = np.round(new_point/spacing, decimals=0)
    # Adjust coords to be in [0,max] for visited matrix rather than center at origin (already checked bounds so this should be ok)
    #if x_dis<0:  x_dis+=w_dis//2
    #if y_dis<0:  y_dis+=h_dis//2
    x_dis = int(new_point[0]) + w_dis//2
    y_dis = int(new_point[1]) + h_dis//2
    if visited_matrix[x_dis, y_dis, action_id]:
        print(">> Point already visited: ", Xn,Yn,ThetaDeg)
        return [],[]
    else:
        visited_matrix[x_dis, y_dis, action_id] = True
        
    # Note:  append the originally calculated point, not the rounded version
    new_positions.append([Xn,Yn])
    thetas.append(ThetaDeg)

    return new_positions, thetas



### Plot FROM parent TO node at node angle (angle of arrival)
# Change this to draw curves??
def plot_vector(node, c='k', w=0.025):
    if node.parent==None:  return
    # Adjust for origin
    x=node.parent.coord[0]+w//2
    y=node.parent.coord[1]+h//2

    d = distance_2(node.parent.coord,node.coord)
    rad = np.deg2rad(node.theta)
    q = d*np.cos(rad)
    v = d*np.sin(rad)

    # Plot vector
    ax.quiver(x, y, q, v, units='xy', angles='xy', scale=1, color=c, width=w)


def get_gscore(previous,current):
    return (previous.g + distance_2(previous.coord, current))
    # Calculate g incrementally, so angle of approach is not needed

def get_hscore(current):
    return distance_2(current, goal_point)


##actions=[[0,RPM1],[RPM1,0],[RPM1,RPM1],[0,RPM2],[RPM2,0],[RPM2,RPM2],[RPM1,RPM2],[RPM2,RPM1]]


def graph_search(start_point,goal_point):
    #actions=[[0,RPM_L],[RPM_L,0],[RPM_L,RPM_L],[0,RPM_R],[RPM_R,0],[RPM_R,RPM_R],[RPM_L,RPM_R],[RPM_R,RPM_L]]
    u_L = convert_RPM_mmps(RPM_L)
    u_R = convert_RPM_mmps(RPM_R)
    actions=[ [0,u_L], [u_L,0], [u_L,u_L], [0,u_R], [u_R,0], [u_R,u_R], [u_L,u_R], [u_R,u_L] ]


    start_node = Node(0, start_point, g=0, h=starth, f=0+starth, theta=theta_s) 
    node_q = [start_node]  # put the startNode on the openList with f=0
    explored_nodes = [] # points visited
    #child_nodes = []  # closed list
    ##final_nodes.append(node_q[0])  # Only writing data of nodes in seen
    ##visited_coord.append(node_q[0].coord)
    node_counter = 0  # To define a unique ID to all the nodes formed
    action_count=0

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
        print("> Current node & dist: ", current_root.coord, current_root.theta, current_root.f)
        # Incorporate radius for reaching goal (currently 100mm)
        coord_min = [current_root.coord[0]-spacing, current_root.coord[1]-spacing]
        coord_max = [current_root.coord[0]+spacing, current_root.coord[1]+spacing]
        if coord_min[0]<=goal_point[0]<=coord_max[0] and coord_min[1]<=goal_point[1]<=coord_max[1]:
            print("\nGoal reached:  ", current_root.coord, current_root.theta, current_root.f)
            return current_root
        
        for action_id,action in enumerate(actions):
            action_count=action_count+1        
            child_coords,thetas = generate_node_successor(current_root.coord,current_root.theta,action,action_id)
            # Having issues when no children found so check that here
            if len(child_coords)==0 or len(thetas)==0:
                continue    
            for child_point,theta in zip(child_coords, thetas):  # Redundant line, since generate_node_successor now returns one child; remove later
                #print("child_point: ", child_point, theta)
                node_counter+=1
                #print("node count: ", node_counter)
                tempg=get_gscore(current_root,child_point)
                temph=get_hscore(child_point)
                child_node = Node(node_counter, child_point, parent=current_root, g=tempg, h=temph, f=tempg+temph, theta=theta)

                #child_nodes.append(child_node)
            # Redundant line, removing for efficiency
            #for child in child_nodes:
                # Adjusted this to replace explored nodes if the node is found again with a lower cost ###
                for i,explored in enumerate(explored_nodes):
                    if child_node.coord[1]==explored.coord[0] and child_node.coord[1]==explored.coord[1] and child_node.g<explored.g:
                        print("Reached previously explored node with lower cost, replacing")
                        explored_nodes[i] = child_node
                        #continue
                for item in node_q:
                    if (child_node.coord==item.coord) and child_node.g>item.g:
                        print("Coordinates present with lower cost, not adding to queue")
                        continue
                node_q.append(child_node)

            print("node count: ", node_counter, "action count: ", action_count)



def find_path(node):  # To find the path from the goal node to the starting node
    if node==None:
        print(">> Error:  Goal node not found")
        return

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

plt.show()

