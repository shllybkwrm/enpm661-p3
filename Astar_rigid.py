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


##### Input functions #####
def get_parameters():
    print("Please enter the rigid robot parameters.")
    ans=(input("Enter the radius (default=3): "))
    if ans=='':  radius=3
    else:  radius=int(ans)
    ans=(input("Enter the obstacle clearance (default=2): "))
    if ans=='':  clearance=2
    else:  clearance=int(ans)
    ans=(input("Enter the robot step size (1-10, default=1): "))
    if ans=='' or int(ans)<1:  step=1
    elif int(ans)>10:  step=10
    else:  step=int(ans)

    return radius, clearance, step

def get_start():
    print("\nPlease enter the initial coordinates of the robot.")
    ans=(input("Enter the x coordinate (default=50): "))
    if ans=='':  x=50
    else:  x=int(ans)
    ans=(input("Enter the y coordinate (default=30): "))
    if ans=='':  y=30
    else:  y=int(ans)
    ans=(input("Enter the starting theta (30-deg increments, default=60): "))
    if ans=='':  theta_s=60
    else:  theta_s=int(ans)

    return [x, y], theta_s

def get_goal():
    print("\nPlease enter the coordinates of the robot's goal.")
    ans=(input("Enter the target x coordinate (default=150): "))
    if ans=='':  x=90
    else:  x=int(ans)
    ans=(input("Enter the target y coordinate (default=150): "))
    if ans=='':  y=40
    ans=(input("Enter the goal theta (30-deg increments, default=same as start): "))
    if ans=='':  theta_g=theta_s
    else:  theta_g=int(ans)

    return [x, y], theta_g


# Get input parameters
radius, clearance, step = get_parameters()  # NOTE:  Still need to incorporate clearance ###
start_point, theta_s = get_start()
goal_point, theta_g = get_goal()
print()


########### SET ROBOT COORDINATE ##############
robot_x_coord=start_point[0]
robot_y_coord=start_point[1]
goal_x_coord=goal_point[0]
goal_y_coord=goal_point[1]
robot_height=radius
robot_breadth=radius
x=np.linspace((robot_x_coord-robot_breadth/2),(robot_x_coord+robot_breadth/2),robot_breadth+1,dtype=int)
y=np.linspace((robot_y_coord+robot_height/2),(robot_y_coord-robot_height/2),robot_height+1,dtype=int)
x_1,y_1=np.meshgrid(x,y, indexing='xy')
points = np.array(list(zip(x_1.flatten(),y_1.flatten())))
########### PLOTTING THE ROBOT - change to circle? ################
rcodes = [Path.MOVETO] + [Path.LINETO]*3 + [Path.CLOSEPOLY]
rvertices = [((robot_x_coord-robot_breadth/2), (robot_y_coord-robot_height/2)), ((robot_x_coord-robot_breadth/2), (robot_y_coord+robot_height/2)), ((robot_x_coord+robot_breadth/2), (robot_y_coord+robot_height/2)), ((robot_x_coord+robot_breadth/2), (robot_y_coord-robot_height/2)), (0, 0)]
robot = Path(rvertices,rcodes)
robotpatch = PathPatch(robot, facecolor='green', edgecolor='green')
########### PLOTTING THE GOAL - change to circle? ################
gcodes = [Path.MOVETO] + [Path.LINETO]*3 + [Path.CLOSEPOLY]
gvertices = [((goal_x_coord-robot_breadth/2), (goal_y_coord-robot_height/2)), ((goal_x_coord-robot_breadth/2), (goal_y_coord+robot_height/2)), ((goal_x_coord+robot_breadth/2), (goal_y_coord+robot_height/2)), ((goal_x_coord+robot_breadth/2), (goal_y_coord-robot_height/2)), (0, 0)]
goal = Path(gvertices,gcodes)
goalpatch = PathPatch(goal, facecolor='red', edgecolor='red')
########### Prepare Searchable Nodes ################
xm=np.linspace(0,w,num=w,endpoint=False,dtype=int)
ym=np.linspace(h,0,num=h,endpoint=False,dtype=int)
x_m,y_m=np.meshgrid(xm,ym)
map=np.array(list(zip(x_m.flatten(),y_m.flatten())))
cost=(((robot_x_coord-x_m)**2)+((robot_y_coord-y_m)**2))**(0.5)
##print(map.shape)
# PREPARE OBSTACLE SPACE
line1x=((x_m<=25)&(x_m>=20))
line1y=(y_m<=((120+(13*(x_m-20)))))
line2x=((x_m<=75)&(x_m>=25))
line2y=(y_m<=185)
line3x=((x_m<=50)&(x_m>=20))
line3y=(y_m>=((((65/55)*(x_m-20))+120)))
line4x=((x_m<=100)&(x_m>=75))
line4y=(y_m<=(((35/25)*(100-x_m))+150))
line5x=((x_m<=100)&(x_m>=75))
line5y=(y_m>=((30/25)*(x_m-75))+120)
line6x=((x_m<=75)&(x_m>=50))
line6y=(y_m>=((30/25)*(75-x_m))+120)
line7x=((x_m<=95)&(x_m>=30))
line7y=(y_m>=0.57763130100345*(95-x_m)+30)
line8x=((x_m<=105)&(x_m>=95))
line8y=(y_m>=1.73373802511719*(x_m-95)+30)
def obstacle_halfplane(map):
    for i in range(0,len(map)):
        ox=map[i][0]
        oy=map[i][1]
        crown=np.where(20<=ox>=25)
        print(oy)
########### PLOTTING THE MAP SPACE #############
vertices = []
codes = []
######## POLYGON OBSTACLES ###################
codes = [Path.MOVETO] + [Path.LINETO]*5 + [Path.CLOSEPOLY]
vertices = [(20, 120), (25, 185), (75, 185), (100, 150), (75,120),(50,150),(0, 0)]
codes += [Path.MOVETO] + [Path.LINETO]*3 + [Path.CLOSEPOLY]
vertices += [(30, 67.54603), (40, 84.88341025), (105,47.32472), (95, 35), (0, 0)]
codes += [Path.MOVETO] + [Path.LINETO]*3 + [Path.CLOSEPOLY]
vertices += [(200, 25), (225,40), (250,25), (225,10), (0, 0)]
vertices = np.array(vertices, float)
path = Path(vertices, codes)
####### CIRCLE OBSTACLE #####################
circle=Path.circle((225,150),radius=25,readonly=False)
####### ELIPSE OBSTACLE ####################
ellipse=Ellipse((150,100),80,40,0,facecolor='None', edgecolor='blue')
###### PATCHING THEM TOGETHER ###############
pathpatch = PathPatch(path, facecolor='None', edgecolor='blue')
pathpatch2 = PathPatch(circle, facecolor='None', edgecolor='blue')
pathpatch3= Ellipse((150,100),80,40,0,facecolor='None', edgecolor='blue')

####### CHECKING TO SEE IF ROBOT IS IN OBSTACLE ################
def inside_obstacle(points):
    inside_polygons = (path.contains_points(points, radius=1e-9))#true if it is inside the polygon,otherwise false
    inside_ellipse= (ellipse.contains_points(points,radius=1e-9))
    inside_circle= (circle.contains_points(points,radius=1e-9))
    return (any(inside_polygons==True)) or (any(inside_circle==True)) or (any(inside_ellipse==True))
if inside_obstacle(points):
    print("error:  robot or goal starts inside obstacle!")
    exit()
####################### REDUCING SEACH NODES BY SUBTRACTING OBSTACLES ################
inside_polygons2 = np.where((path.contains_points(map, radius=1e-9)),20000,0)
inside_ellipse2= np.where((ellipse.contains_points(map,radius=1e-9)),20000,0)
inside_circle2= np.where((circle.contains_points(map,radius=1e-9)),20000,0)
mask=(inside_polygons2+inside_ellipse2+inside_circle2)
reduced=np.array(list(zip(x_m.flatten(),y_m.flatten(),mask.flatten())))
reduced2=np.vstack((x_m.flatten(),y_m.flatten(),mask)).T
newmap=reduced[np.all(reduced<20000, axis=1),:]# if any value in reduced map is True remove it
newmap=np.delete(newmap,2,axis=1)

###### PLOTTING #####################
fig, ax = plt.subplots()
ax.add_patch(robotpatch)
ax.add_patch(goalpatch)
ax.add_patch(pathpatch)
ax.add_patch(pathpatch2)
ax.add_patch(pathpatch3)
ax.set_title('Map Space')
print(ax)
ax.autoscale_view()
plt.xlim(0,w)
plt.ylim(0,h)
plt.show()



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

#def get_and_replace_min_cost(node):
#    for i in node_q:
#        costfinder={i.cost:i}
#        k=min(costfinder.keys())
#        node_q.remove(costfinder[k])
#        node_q.insert(0,costfinder[k])
#        return node_q

# No need to re-create this every time
degree_list=np.linspace(0, 360, 12, endpoint=False, dtype=int)
# Map for duplicate checking
visited_matrix = np.zeros((w*2,h*2,12), dtype=bool)
i = np.where(degree_list==theta_s)
visited_matrix[start_point[0], start_point[1], i] = True

# Still needs collision and bound checking
def generate_node_successor(coord):
    new_positions=[]
    thetas=[]
    #updated=[]
    for i,angle in enumerate(degree_list):
        # NOTE:  Incorporated robot step size here
        rad = np.deg2rad(angle)
        new_point = np.array([(coord[0] + step*np.cos(rad)), (coord[1] + step*np.sin(rad))])
        new_point = ( np.round(new_point*2, decimals=0) ) / 2
    
        # Check for duplicates
        a = int(new_point[0]*2)
        b = int(new_point[1]*2)
        #deg = int(np.rad2deg(t))
        if a<0 or a>w*2:
            continue
        if b<0 or b>h*2:
            continue
        if inside_obstacle():  # NOTE:  Won't work until points are drawn??
            continue
        if visited_matrix[a, b, i]:
            #print("node already visited: ", new_point, angle)
            continue
        #else:
        visited_matrix[a, b, i] = True
        new_positions.append(new_point)
        thetas.append(angle)
    
    # This shouldn't be needed with the new duplicate checking?
#    for i in range(0,len(new_positions)):
#        for j in range(1,(len(new_positions)-1)):
#            distance=distance_2(new_positions[j],new_positions[i])
###            print("dis",distance)
#        if distance>0.5:
#            updated.insert(0,new_positions[i]) 
#    return updated

    return new_positions, thetas


def get_gscore(previous,current):
    return (previous.g + distance_2(previous.coord, current))
    # Calculate g incrementally, so angle of approach is not needed

def get_hscore(current):
    return distance_2(current, goal_point)



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
        print("current node: ", current_root.coord, current_root.theta, current_root.f)
        if current_root.coord[0]==goal_point[0] and current_root.coord[1]==goal_point[1]:# and current_root.theta==theta_g:
            print("\nGoal reached:  ", current_root.coord, current_root.theta, current_root.f)
            return current_root

        child_coords,thetas = generate_node_successor(current_root.coord)
        # Having issues when no children found so check that here
        if child_coords==[]:
            continue    
        for child_point,theta in zip(child_coords, thetas):
            #print("child_point: ", child_point, theta)
            node_counter+=1
            #print("node count: ", node_counter)
            tempg=get_gscore(current_root,child_point)
            temph=get_hscore(child_point)
            child_node = Node(node_counter, child_point, parent=current_root, g=tempg, h=temph, f=tempg+temph, theta=theta)
            #child_nodes.append(child_node)
        # Redundant line, removing for efficiency
        #for child in child_nodes:
            # Probably not needed because of visited matrix
            # Not sure this actually works anyway because of object addresses
            #for explored in explored_nodes:
            #    if child_node==explored:
            #        continue
            for item in node_q:
                if child_node.coord[1]==item.coord[0] and child_node.coord[1]==item.coord[1] and child_node.g>item.g:
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

    return list(reversed(p))



print("\nThe start is", start_point, theta_s)
print("The goal is", goal_point, theta_g)
starth = get_hscore(start_point)
print("The distance to goal is", starth, '\n')

start_time = time.time()

goal_node=graph_search(start_point,goal_point)
result=find_path(goal_node)
print(result)

end_time = time.time()
print("Total execution time:", end_time-start_time)

