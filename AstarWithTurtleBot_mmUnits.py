# ENPM661 Project 3 Phase 3
# Shelly Bagchi & Omololu Makinde

import math
import time
import numpy as np
from matplotlib.path import Path
from matplotlib.patches import PathPatch
from matplotlib.patches import Rectangle
from matplotlib.collections import PatchCollection
import matplotlib.pyplot as plt


# Note:  Values now in mm - original map given in meters
# The outer square is 10,200mm but I only used the inner
w = 10000
h = 10000
rob_radius=354//2
w_radius=76//2
w_circ = np.pi*76

# TB2 max speed:  650 mm/s
max_RPM = (650*60)/w_circ
def convert_RPM_mmps (RPM):
    V =  (RPM/60)*w_circ
    if V>650: return 650
    elif V<0: return 0
    else:  return V


####IF WE HAVE TIME WE SHOULD CHANGE THIS INTO A GUI AND GET ALL INPUTS ONE TIME
##### Input functions #####
def get_parameters():
    print("Enter the robot settings (note: robot may overshoot the goal at large RPMs).  All other parameters are used from the TurtleBot 2.")
##    ans=(input("Enter the radius (default=3): "))
##    if ans=='':  radius=3
##    else:  radius=int(ans)
##    ans=(input("Enter the robot step size (1-10, default=1): "))
##    if ans=='' or int(ans)<1:  step=1
##    elif int(ans)>10:  step=10
##    else:  step=int(ans)
    ans=(input("Enter the left wheel speed in RPM (default=50, max=%.2f): "  %max_RPM))
    if ans=='':  RPM_L=50
    else:  RPM_L=int(ans)
    ans=(input("Enter the right wheel speed in RPM (default=50, max=%.2f): " %max_RPM))
    if ans=='':  RPM_R=50
    else:  RPM_R=int(ans)
    ans=(input("Enter the obstacle clearance in mm (default=10): "))
    if ans=='':  clearance=10
    else:  clearance=int(ans)

##    return radius, clearance, step, RPM1, RPM2
    return clearance, RPM_L, RPM_R

def get_start():
    print("\nEnter the initial coordinates of the robot.  The map origin is at the center.")
    ans=(input("Enter the x coordinate in mm (default=-4000): "))
    if ans=='':  x=-4000
    else:  x=int(ans)
    ans=(input("Enter the y coordinate in mm (default=-3000): "))
    if ans=='':  y=-3000

    else:  y=int(ans)
    ans=(input("Enter the starting theta in degrees (default=45): "))
    if ans=='':  theta_s=45
    else:  theta_s=int(ans)

    return [x, y], theta_s

def get_goal():
    print("\nEnter the coordinates of the robot's goal.")
    ans=(input("Enter the target x coordinate in mm (default=4000): "))
    if ans=='':  x=4000
    else:  x=int(ans)
    ans=(input("Enter the target y coordinate in mm (default=3000): "))
    if ans=='':  y=3000
    else:  y=int(ans)

    return [x, y]


def drotmatrix(point,angle):
    R=np.array([[np.cos(np.deg2rad(angle)),-(np.sin(np.deg2rad(angle)))],[np.sin(np.deg2rad(angle)),np.cos(np.deg2rad(angle))]])
    b=np.array(point).transpose()
##    b=np.array(point)
    #print(np.dot(R,b))
##    print(np.dot(b,R))
    return np.dot(R,b)


# Get input parameters
clearance, RPM_L, RPM_R = get_parameters()  #Changed from proj 3-2
start_point, theta_s = get_start()
goal_point = get_goal()  #Changed from proj 3-2
print()


# NOTE:  Map goes from negative to positive with origin at center.  No need to change coords.

########### SET ROBOT COORDINATE ##############
### Adjusted to be in matplotlib coords
robot_x_coord=start_point[0]#+w//2
robot_y_coord=start_point[1]#+h//2
goal_x_coord=goal_point[0]#+w//2
goal_y_coord=goal_point[1]#+h//2

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


############# PLOTTING THE ROBOT - changed to circle, can see rotation from plotted curves ################
#rcodes = [Path.MOVETO] + [Path.LINETO]*3 + [Path.CLOSEPOLY]
#rvertices = [((robot_x_coord-robot_breadth/2), (robot_y_coord-robot_height/2)), ((robot_x_coord-robot_breadth/2), (robot_y_coord+robot_height/2)), ((robot_x_coord+robot_breadth/2), (robot_y_coord+robot_height/2)), ((robot_x_coord+robot_breadth/2), (robot_y_coord-robot_height/2)), (0, 0)]
#robot = Path(rvertices,rcodes)
robot = Path.circle((robot_x_coord,robot_y_coord), radius=rob_radius)
robotpatch = PathPatch(robot, facecolor='green', edgecolor='green')
#print("vertices ",rvertices,"shape ", np.shape(rvertices))
#rotvertices=[]
#for i in np.array(rvertices):
#    i=np.array(i)
#    t=i-rvertices[0]
#    #print("t",t)
#    roti=drotmatrix(t,theta_s)
#    rotvertices.append((roti+rvertices[0]))
###print("new vertices ",rotvertices,"shape ", np.shape(rotvertices))
#rotcodes = [Path.MOVETO] + [Path.LINETO]*3 + [Path.CLOSEPOLY]
#rotrobot = Path(rotvertices,rotcodes)
#rotrobotpatch = PathPatch(rotrobot, facecolor='blue', edgecolor='blue')

########### PLOTTING THE GOAL - changed to circle ################
#gcodes = [Path.MOVETO] + [Path.LINETO]*3 + [Path.CLOSEPOLY]
#gvertices = [((goal_x_coord-robot_breadth/2), (goal_y_coord-robot_height/2)), ((goal_x_coord-robot_breadth/2), (goal_y_coord+robot_height/2)), ((goal_x_coord+robot_breadth/2), (goal_y_coord+robot_height/2)), ((goal_x_coord+robot_breadth/2), (goal_y_coord-robot_height/2)), (0, 0)]
#goal = Path(gvertices,gcodes)
goal = Path.circle((goal_x_coord,goal_y_coord), radius=rob_radius)
goalpatch = PathPatch(goal, facecolor='red', edgecolor='red')


########## CIRCLE OBSTACLES #####################
#circle1=Path.circle((5000,5000),radius=1000,readonly=False)
#circle2=Path.circle((3000,2000),radius=1000,readonly=False)
#circle3=Path.circle((7200,2000),radius=1000,readonly=False)
#circle4=Path.circle((7200,8000),radius=1000,readonly=False)
circle1=Path.circle((0,0),radius=1000,readonly=False)
circle2=Path.circle((-2000,-3000),radius=1000,readonly=False)
circle3=Path.circle((2000,-3000),radius=1000,readonly=False)
circle4=Path.circle((2000,3000),radius=1000,readonly=False)
pathpatch1 = PathPatch(circle1, facecolor='None', edgecolor='blue')
pathpatch2 = PathPatch(circle2, facecolor='None', edgecolor='blue')
pathpatch3 = PathPatch(circle3, facecolor='None', edgecolor='blue')
pathpatch4 = PathPatch(circle4, facecolor='None', edgecolor='blue')
#circlepatches = PatchCollection([circle1,circle3,circle3,circle4], facecolor='None', edgecolor='blue')
############# RECTANGULAR OBSTACLES #############
# Only for drawing
square1 = Rectangle((-2500,2000), 1500, 1500, fill=False)
square2 = Rectangle((-4750,-1000), 1500, 1500, fill=False)
square3 = Rectangle((3250,-1000), 1500, 1500, fill=False)
squarepatches = PatchCollection([square1,square2,square3], facecolor='None', edgecolor='blue')
########## POLYGON OBSTACLES ###################
# Only for obstacle check - for some reason the other version didn't work
codes = [Path.MOVETO] + [Path.LINETO]*3 + [Path.CLOSEPOLY]
vertices = [(-2500,2000), (-2500,3500), (-1000,3500), (-1000,2000),(0, 0)]
codes += [Path.MOVETO] + [Path.LINETO]*3 + [Path.CLOSEPOLY]
vertices += [(-4750,-1000), (-4750,500), (-3250,500), (-3250,-1000), (0, 0)]
codes += [Path.MOVETO] + [Path.LINETO]*3 + [Path.CLOSEPOLY]
vertices += [(3250,-1000), (3250,500), (4750,500), (4750,-1000), (0, 0)]
vertices = np.array(vertices, float)
polygon_path = Path(vertices, codes)
#polygon_pathpatch = PathPatch(polygon_path, facecolor='None', edgecolor='blue')


####### CHECKING TO SEE IF ROBOT IS IN OBSTACLE ################
def inside_obstacle(points):
    effective_clearance = clearance+rob_radius
    inside_circle1= (circle1.contains_points(points,radius=effective_clearance))
    inside_circle2= (circle2.contains_points(points,radius=effective_clearance))
    inside_circle3= (circle3.contains_points(points,radius=effective_clearance))
    inside_circle4= (circle4.contains_points(points,radius=effective_clearance))
    inside_square1= (square1.get_path().contains_points(points,radius=effective_clearance))
    inside_square2= (square2.get_path().contains_points(points,radius=effective_clearance))
    inside_square3= (square3.get_path().contains_points(points,radius=effective_clearance))
    inside_polygons = (polygon_path.contains_points(points, radius=effective_clearance))
    return (all(inside_circle1==True) or all(inside_circle2==True) or all(inside_circle3==True) or all(inside_circle4==True) or all(inside_square1==True) or all(inside_square2==True) or all(inside_square3==True) or all(inside_polygons==True) )

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
plt.xlim(-w/2,w/2)
plt.ylim(-h/2,h/2)
#ax.autoscale_view()
ax.add_patch(robotpatch)
ax.add_patch(goalpatch)
ax.add_patch(pathpatch1)
ax.add_patch(pathpatch2)
ax.add_patch(pathpatch3)
ax.add_patch(pathpatch4)
#ax.add_collection(circlepatches)
ax.add_collection(squarepatches)
#ax.add_patch(polygon_pathpatch)
ax.set_aspect('equal')
ax.set_title('Map Space (units=mm)')
#plt.show()


####################### A STAR ################
class Node:
    def __init__(self, node_no, coord, theta=0, parent=None, g=0, h=0, f=0):
        self.node_no = node_no
        self.parent = parent
        self.coord = coord
        self.theta = theta
        self.g=g
        self.h=h
        self.f=f

    def __repr__(self):
        return repr((self.node_no, self.coord, self.theta, self.f))


def distance_2(p1,p2):
    distance = math.sqrt( ((p1[0]-p2[0])**2)+((p1[1]-p2[1])**2) )
    return distance


### Map for duplicate checking
# Dicretize space; action space is now 8 not 12
spacing=500
w_dis = w//spacing
h_dis = h//spacing
visited_matrix = np.zeros((w_dis,h_dis,8), dtype=bool)

degree_list=np.linspace(0, 360, 8, endpoint=False, dtype=int)
i = np.where(degree_list==theta_s)
visited_matrix[start_point[0]//spacing, start_point[1]//spacing, i] = True

# Note:  Function updated with corrections from new TA code
def generate_node_successor(coord,thetaIn,action,action_id):
    new_positions=[]
    thetas=[]

    r=w_radius
    L=rob_radius
    t=0.0
    dt=0.1  # e.g. 10 steps
    Xn=coord[0]
    Yn=coord[1]
    Thetan=np.deg2rad(thetaIn)

    ### Convert RPM to rad/sec
    UL = action[0]*(1/60)*2*np.pi
    UR = action[1]*(1/60)*2*np.pi

# Xs, Ys: Start point coordinates for plot function
# Xn, Yn, Thetan: End point coordintes

    while t<1:
        t=t+dt
        Xs = Xn
        Ys = Yn

        # Note:  These are using the rotational velocities of the wheels in rad/sec
        Xn += 0.5 * r * (UL + UR) * math.cos(Thetan) * dt
        Yn += 0.5 * r * (UL + UR) * math.sin(Thetan) * dt
        Thetan += (r / L) * (UR - UL) * dt

        # Check collisions at every point on path
        if inside_obstacle([[Xn,Yn]]):
            #print(">> Collides with obstacle!")
            #return [],[]
            # Make sure this new point isn't included (roll back changes)
            Xn = Xs
            Yn = Ys
            break

        # Plot here to get a curve rather than vector
        #ax.quiver(Xs, Ys, Xn-Xs, Yn-Ys, units='xy', scale=1, color='k', width=1, headwidth=1, headlength=0)
        ax.plot([Xs, Xn], [Ys, Yn], linewidth=0.25, color='k')

    ThetaDeg=np.rad2deg(Thetan) % 360
    #print("New child:  (%.2f, %.2f), %.2f deg" %(Xn,Yn,ThetaDeg), "from action", action_id)

    # Check collisions again just in case?
    if inside_obstacle([[Xn,Yn]]):
        #print(">> Collides with obstacle!")
        return [],[]

    ### Check bounds accounting for origin at center
    if Xn<=-w/2 or Xn>=w/2:
        #print(">> Out of bounds")
        return [],[]
    if Yn<=-h/2 or Yn>=h/2:
        #print(">> Out of bounds")
        return [],[]

    ### Check for duplicates
    # Discretize to grid
    new_point = np.round(np.array([Xn,Yn])/spacing, decimals=0)
    # Adjust coords to be in [0,max] for visited matrix rather than center at origin (already checked bounds so this should be ok)
    #if x_dis<0:  x_dis+=w_dis//2
    #if y_dis<0:  y_dis+=h_dis//2
    x_dis = int(new_point[0]) + w_dis//2 -1
    y_dis = int(new_point[1]) + h_dis//2 -1
    # Find closest angle in degree list
    temp = (degree_list -ThetaDeg )**2
    degree_id = np.argmin(temp)
    #if visited_matrix[x_dis, y_dis, action_id]:
    if visited_matrix[x_dis, y_dis, degree_id]:
        #print(">> Child already visited: ", x_dis,y_dis,action_id)
        return [],[]
    else:
        visited_matrix[x_dis, y_dis, degree_id] = True

    # Note:  append the originally calculated point, not the discretized version
    # Should we round these to ints??
    new_positions.append([Xn,Yn])
    thetas.append(ThetaDeg)
    #print("New child:  (%.2f, %.2f), %.2f deg" %(Xn,Yn,ThetaDeg), "from action", action_id)

    #plt.show(block=False)
    #plt.pause(0.01)

    return new_positions, thetas



def get_gscore(previous,current):
    return (previous.g + distance_2(previous.coord, current))
    # Calculate g incrementally, so angle of approach is not needed

def get_hscore(current):
    return distance_2(current, goal_point)


##actions=[[0,RPM1],[RPM1,0],[RPM1,RPM1],[0,RPM2],[RPM2,0],[RPM2,RPM2],[RPM1,RPM2],[RPM2,RPM1]]


def graph_search(start_point,goal_point):
    actions=[[0,RPM_L],[RPM_L,0],[RPM_L,RPM_L],[0,RPM_R],[RPM_R,0],[RPM_R,RPM_R],[RPM_L,RPM_R],[RPM_R,RPM_L]]
    #u_L = convert_RPM_mmps(RPM_L)
    #u_R = convert_RPM_mmps(RPM_R)
    #actions=[ [0,u_L], [u_L,0], [u_L,u_L], [0,u_R], [u_R,0], [u_R,u_R], [u_L,u_R], [u_R,u_L] ]
    #print("Action set: ", actions, '\n')


    start_node = Node(0, start_point, theta=theta_s, g=0, h=starth, f=0+starth)
    node_q = [start_node]  # put the startNode on the openList with f=0
    explored_nodes = [] # points visited
    #child_nodes = []  # closed list
    ##final_nodes.append(node_q[0])  # Only writing data of nodes in seen
    ##visited_coord.append(node_q[0].coord)
    node_counter = 0  # To define a unique ID to all the nodes formed
    action_count=0

    ##for i in range(1):#while node_q:  # UNCOMMENT FOR DEBUGGING
    while node_q: #while the OPEN list is not empty
        #current_root = node_q[0]##############################change current root to equal node with smallest f value#############
        #current_index = 0
        #for index,thing in enumerate(node_q):#let the currentNode equal the node with the lowest cost
        #    if thing.f < current_root.f:
        #        current_root = thing
        #        current_index = index
        #node_q.pop(current_index)
        
        # Sort queue by f-score
        node_q = sorted(node_q, key=lambda node: node.f)
        current_root = node_q.pop(0)
        explored_nodes.append(current_root)
        # Should we plot which points are explored from?
        print("> Exploring node (%.2f, %.2f) %.2f deg with score %.2f" %(current_root.coord[0], current_root.coord[1], current_root.theta, current_root.f))

        # Animate plot
        plt.show(block=False)
        plt.pause(0.001)

        # Incorporate radius for reaching goal - the radius around the goal point, plus accounting for the robot's own radius
        coord_min = [current_root.coord[0]-rob_radius*2, current_root.coord[1]-rob_radius*2]
        coord_max = [current_root.coord[0]+rob_radius*2, current_root.coord[1]+rob_radius*2]
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
                child_node = Node(node_counter, child_point, theta=theta, parent=current_root, g=tempg, h=temph, f=tempg+temph)

                #child_nodes.append(child_node)
                
                ### Commenting this for now for speed, not sure if this should be done or not
                # Adjusted this to replace explored nodes if the node is found again with a lower cost ###
                #for i,explored in enumerate(explored_nodes):
                #    if child_node.coord[1]==explored.coord[0] and child_node.coord[1]==explored.coord[1] and child_node.g<explored.g:
                #        #print("Reached previously explored node with lower cost, replacing...")
                #        explored_nodes[i] = child_node
                        #continue

                for item in node_q:
                    if (child_node.coord==item.coord) and child_node.g>item.g:
                        #print("Coordinates present with lower cost-to-come, not adding to queue.")
                        continue
                node_q.append(child_node)

            #print("node count: ", node_counter, "action count: ", action_count)

    print(">>>Error:  Queue emptied without finding goal (probably no valid children generated)")
    return None


### Plot FROM parent TO node at node angle (angle of arrival)
# Change this to draw curves??
def plot_vector(node, c='k', w=0.3):
    x2=node.coord[0]
    y2=node.coord[1]
    ax.scatter(x2, y2, s=w*2, color=c)

    if node.parent==None:  return
    x1=node.parent.coord[0]
    y1=node.parent.coord[1]

    #d = distance_2(node.parent.coord,node.coord)
    #rad = np.deg2rad(node.theta)
    #q = d*np.cos(rad)
    #v = d*np.sin(rad)
    q = x2-x1
    v = y2-y1

    # Plot vector
    ax.quiver(x1, y1, q, v, units='xy', angles='xy', scale=1, color=c, width=w, headwidth=w, headlength=w*2)


def find_path(node):  # To find the path from the goal node to the starting node
    if node==None:
        print(">>> Error:  Goal node not found")
        return

    p = []
    p.append(node)
    parent_node = node.parent
    while parent_node is not None:
        p.append(parent_node.coord)
        plot_vector(parent_node, c='b', w=10)
        parent_node = parent_node.parent

    return list(reversed(p))



print("\nThe RPMs are", RPM_L, RPM_R)
print("The start is", start_point, theta_s)
print("The goal is", goal_point)
starth = get_hscore(start_point)
print("The distance to goal is", starth, '\n')

start_time = time.time()

goal_node=graph_search(start_point,goal_point)
result=find_path(goal_node)
print('\n', result)

end_time = time.time()
print("Total execution time:", end_time-start_time)

plt.show()
