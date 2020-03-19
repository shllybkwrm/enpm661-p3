import numpy as np
from matplotlib.path import Path
from matplotlib.patches import PathPatch
from matplotlib.patches import Ellipse
import matplotlib.pyplot as plt

###########GET ROBOT COORDINATE##############
robot_x_coord=150
robot_y_coord=100
robot_height=10
robot_bredth=6
x=np.linspace((robot_x_coord-robot_bredth/2),(robot_x_coord+robot_bredth/2),robot_bredth+1,dtype=int)
y=np.linspace((robot_y_coord+robot_height/2),(robot_y_coord-robot_height/2),robot_height+1,dtype=int)
x_1,y_1=np.meshgrid(x,y, indexing='xy')
points = np.array(list(zip(x_1.flatten(),y_1.flatten())))
###########PLOTTING THE ROBOT################
rcodes = [Path.MOVETO] + [Path.LINETO]*3 + [Path.CLOSEPOLY]
rvertices = [((robot_x_coord-robot_bredth/2), (robot_y_coord-robot_height/2)), ((robot_x_coord-robot_bredth/2), (robot_y_coord+robot_height/2)), ((robot_x_coord+robot_bredth/2), (robot_y_coord+robot_height/2)), ((robot_x_coord+robot_bredth/2), (robot_y_coord-robot_height/2)), (0, 0)]
robot = Path(rvertices,rcodes)
robotpatch = PathPatch(robot, facecolor='green', edgecolor='green')
###########Prepare Searchable Nodes
xm=np.linspace(0,300,301,dtype=int)
ym=np.linspace(200,0,201,dtype=int)
x_m,y_m=np.meshgrid(xm,ym)
map=np.array(list(zip(x_m.flatten(),y_m.flatten())))
cost=(((robot_x_coord-x_m)**2)+((robot_y_coord-y_m)**2))**(0.5)
##print(map.shape)
#PREPARE OBSTACLE SPACE
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
###########PLOTTING THE MAP SPACE#############
vertices = []
codes = []
########POLYGON OBSTACLES###################
codes = [Path.MOVETO] + [Path.LINETO]*5 + [Path.CLOSEPOLY]
vertices = [(20, 120), (25, 185), (75, 185), (100, 150), (75,120),(50,150),(0, 0)]
codes += [Path.MOVETO] + [Path.LINETO]*3 + [Path.CLOSEPOLY]
vertices += [(30, 67.54603), (40, 84.88341025), (105,47.32472), (95, 35), (0, 0)]
codes += [Path.MOVETO] + [Path.LINETO]*3 + [Path.CLOSEPOLY]
vertices += [(200, 25), (225,40), (250,25), (225,10), (0, 0)]
vertices = np.array(vertices, float)
path = Path(vertices, codes)
#######CIRCLE OBSTACLE#####################
circle=Path.circle((225,150),radius=25,readonly=False)
#######ELIPSE OBSTACLE####################
elipse=Ellipse((150,100),80,40,0,facecolor='None', edgecolor='green')
######PATCHING THEM TOGETHER###############
pathpatch = PathPatch(path, facecolor='None', edgecolor='green')
pathpatch2 = PathPatch(circle, facecolor='None', edgecolor='green')
pathpatch3= Ellipse((150,100),80,40,0,facecolor='None', edgecolor='green')

#######CHECKING TO SEE IF ROBOT IS IN OBSTACLE
#######CHECKING TO SEE IF ROBOT IS IN OBSTACLE
inside_polygons = (path.contains_points(points, radius=1e-9))#true if it is inside the polygon,otherwise false
inside_elipse= (elipse.contains_points(points,radius=1e-9))
inside_circle= (circle.contains_points(points,radius=1e-9))
inside_obstacle=(any(inside_polygons==True)) or (any(inside_circle==True)) or (any(inside_elipse==True))
if inside_obstacle:
    print("inside the obstacle you are, you little thing")
#######################REDUCING SEACH NODES BY SUBTRACTING OBSTACLES
inside_polygons2 = np.where((path.contains_points(map, radius=1e-9)),20000,0)
inside_elipse2= np.where((elipse.contains_points(map,radius=1e-9)),20000,0)
inside_circle2= np.where((circle.contains_points(map,radius=1e-9)),20000,0)
mask=(inside_polygons2+inside_elipse2+inside_circle2)
reduced=np.array(list(zip(x_m.flatten(),y_m.flatten(),mask.flatten())))
reduced2=np.vstack((x_m.flatten(),y_m.flatten(),mask)).T
newmap=reduced[np.all(reduced<20000, axis=1),:]# if any value in reduced map is True remove it
newmap=np.delete(newmap,2,axis=1)

######PLOTTING#####################
fig, (ax,ax2) = plt.subplots(2)
ax.add_patch(robotpatch)
ax.add_patch(pathpatch)
ax.add_patch(pathpatch2)
ax.add_patch(pathpatch3)
ax.set_title('Map Space')
print(ax)
ax.autoscale_view()
ax2.grid(which='both', axis='both', linestyle='-', color='k', linewidth=2)
ax2.set_xticks(0,300,301)
ax2.set_yticks(0,200,201)
ax2.grid(True)
plt.xlim(0,300)
plt.ylim(0,200)
plt.grid(True)
plt.show()

#######################A STAR
##class Node:
##    def __init__(self, node_no, coord, parent, cost,g,h):
##        self.coord = coord
##        self.parent = parent
##        self.node_no = node_no
##        self.cost = cost
##        self.g=g
##        self.h=h
##def distance_2(p1,p2):
##    distance = math.sqrt( ((p1[0]-p2[0])**2)+((p1[1]-p2[1])**2) )
##    return distance
##
##def get_and_replace_min_cost(node):
##    for i in node_q:
##        costfinder={i.cost:i}
##        k=min(costfinder.keys())
##        node_q.remove(costfinder[k])
##        node_q.insert(0,costfinder[k])
##        return node_q
##def generate_node_successor(coord):
##    tetha=np.linspace(0,2*np.pi,13)
##    new_positions=[]
##    updated=[]
##    for t in tetha:
##        thing =np.array([(coord[0]+np.cos(t)),(coord[1]+np.sin(t))])
##        new_positions.append(thing)
##    for i in range(0,len(new_positions)):
##        for j in range(1,(len(new_positions)-1)):
##            distance=distance_2(new_positions[j],new_positions[i])
####            print("dis",distance)
##        if distance>1:
##            updated.insert(0,new_positions[i]) 
##    return updated
##
##def get_gscore(start,current):
##    gscore=distance_2(start,current)
##    return gscore
##def get_hscore(end,current):
##    hscore=distance_2(end,current)
##    return hscore
##def get_fscore(current_score,gscore,hscore):
##    fscore=current_score+gscore+hscore
##    return fscore
##
##
##start_point=[0,0]
##goal_node=[0,2]
##start_node=Node(0, start_point,0, 0,0,0)
##node_q=[start_node]#Put node_start in the OPEN list
##visited_coord=[]
##final_nodes = []#closed
##final_nodes.append(node_q[0])  # Only writing data of nodes in seen
##visited_coord.append(node_q[0].coord)
##node_counter = 0  # To define a unique ID to all the nodes formed
####for i in range(1):#while node_q:  # UNCOMMENT FOR DEBUGGING 
##while node_q: #while the OPEN list is not empty
####    node_q=get_and_replace_min_cost(node_q)
##    current_root = node_q.pop(0)
##    currentg=get_gscore(start_node.coord,current_root.coord)
##    currenth=get_hscore(goal_node,current_root.coord)
##    current_cost= get_gscore(start_node.coord,current_root.coord)+ get_hscore(goal_node,current_root.coord)
##    if np.all((current_root.coord) == (goal_node)):
##        print("Goal reached",current_root.coord,current_root.node_no)
##        print(" child_node, final_nodes, close")
##    temp_coords=generate_node_successor(current_root.coord)
##    for temp in temp_coords:
##        print("temp",temp)
##        node_counter+=1
##        print("counter",node_counter)
##        tempg=get_gscore(start_node.coord,temp)
##        temph=get_hscore(goal_node,temp)
##        temp_cost= (get_gscore(start_node.coord,temp))+ (get_hscore(goal_node,temp))
##        child_node = Node(node_counter, temp, current_root, temp_cost,tempg,temph)
##        if child_node not in node_q:
##            if (child_node.g) <=(child_node.cost):
##                node_q.append(child_node)
##            elif child_node in final_nodes:
##                if (child_node.g) <=(child_node.cost):
##                    node_q.append(child_node)
##            else:
##                final_nodes.append(child_node)
##        final_nodes.append(current_root)
