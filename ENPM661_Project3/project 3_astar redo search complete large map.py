import numpy as np
import math

class Node:
    def __init__(self, node_no, map, parent, act, cost):
        self.map = map
        self.parent = parent
        self.act = act
        self.node_no = node_no
        self.cost = cost

def conv_coord_to_row_col(x,y):
    i=int(200-y)
    j=int (x)
    return i,j

def row_col_to_conv_coord(i,j):
    x=int(j)
    y=int (200-i)
    return x,y
###############################This will get X AND Y COORDINATES OF START AND FINISH POINT FROM USER###########00=90
###############################COMMENT OUT WHEN FIXED################
##def get_initial_robcoord():
##    map = np.ones((201,301),dtype=int)
####    print(map)
##    print("Please enter the x and y coordinates of the point robot")
##    x=int(input("Enter the x coordinate "))
##    y=int(input("Enter the y coordinate "))
##    i,j=conv_coord_to_row_col(x,y)
##    map[i,j]=0
##    return x,y,map
##x,y,map1=get_initial_robcoord()
##print("map1",map1,"x",x,"y",y)
##def get_final_robcoord():
##    map = np.ones((201,301),dtype=int)
####    print(map)
##    print("Please enter the x and y coordinates of the point robot")
##    x=int(input("Enter the x coordinate "))
##    y=int(input("Enter the y coordinate "))
##    i,j=conv_coord_to_row_col(x,y)
##    map[i,j]=0
##    return x,y,map
##u,v,map2=get_final_robcoord()
##print("map2",map2,"u",u,"v",v)
###############################RUNNING CODE DURING DEVELOPMENT###########00=90
###############################COMMENT OUT WHEN AUTOMATED################
x=21
y=125
map1=np.ones((201,301),dtype=int)
map1[y,x]=0
map2=np.ones((201,301),dtype=int)
map2[5,7]=0
def get_robcoord(map):
    i,j=np.where(map ==0)
##    print("row and culumn of 0 ", np.where(map ==0))
    x,y=row_col_to_conv_coord(i,j)
##    print("x and y coordinates", x,y)
    return x,y
##x,y=get_robcoord(map)

def check_location(map):
    x,y=get_robcoord(map)
##    robcoord=[]
    if 11>=x>=9 and 6>=y>=4:
        return True
    if (x-16)**2+(y-5)**2 < 1.5**2:
        return True
    else:
##        robcoord=[x,y]
##        map[y,x]=0
        return x,y,map
x,y,map=check_location(map1)


def move_left(map):
    i,j=np.where(map ==0)
    if j == 0:
        return None
    else:
        temp_arr = np.copy(map)
        temp = temp_arr[i, j - 1]
        temp_arr[i,j] = temp
        temp_arr[i, j - 1] = 0
        if check_location(temp_arr)== True:
            return None
        else:
            return temp_arr
##map=move_left(map)
##print("map",map,"x",x,"y",y)
##
def move_right(map):
    i,j=np.where(map ==0)
    if j == 20:
        return None
    else:
        temp_arr = np.copy(map)
        temp = temp_arr[i, j + 1]
        temp_arr[i,j] = temp
        temp_arr[i, j +1] = 0
        if check_location(temp_arr)== True:
            return None
        else:
            return temp_arr
##map=move_right(map)
##print("map",map)
def move_up(map):
    i,j=np.where(map ==0)
    if i == 0:
        return None
    else:
        temp_arr = np.copy(map)
        temp = temp_arr[i-1, j]
        temp_arr[i,j] = temp
        temp_arr[i-1, j] = 0
        if check_location(temp_arr)== True:
            return None
        else:
            return temp_arr
##map=move_up(map)
##print("map",map)    
def move_down(map):
    i,j=np.where(map ==0)
    if i == 9:
        return None
    else:
        temp_arr = np.copy(map)
        temp = temp_arr[i+1, j]
        temp_arr[i,j] = temp
        temp_arr[i+1, j] = 0
        if check_location(temp_arr)== True:
            return None
        else:
            return temp_arr
##map=move_down(map)
##print("map",map) 
def move_left_up_diag(map):
    i,j=np.where(map ==0)
    if i==0 or j == 0:
        return None
    else:
        temp_arr = np.copy(map)
        temp = temp_arr[i-1, j-1]
        temp_arr[i,j] = temp
        temp_arr[i-1, j-1] = 0
        if check_location(temp_arr)== True:
            return None
        else:
            return temp_arr
##map=move_left_up_diag(map)
##print("move_left_up_diag map",map)

def move_left_down_diag(map):
    i,j=np.where(map ==0)
    if i == 9 or j==0:
        return None
    else:
        temp_arr = np.copy(map)
        temp = temp_arr[i+1, j-1]
        temp_arr[i,j] = temp
        temp_arr[i+1, j-1] = 0
        if check_location(temp_arr)== True:
            return None
        else:
            return temp_arr
##map=move_left_down_diag(map)
##print("move_left_down_diag map",map)
##
def move_right_up_diag(map):
    i,j=np.where(map ==0)
    if i==0 or y == 19:
        return None
    else:
        temp_arr = np.copy(map)
        temp = temp_arr[i-1, j+1]
        temp_arr[i,j] = temp
        temp_arr[i-1, j+1] = 0
        if check_location(temp_arr)== True:
            return None
        else:
            return temp_arr
##map=move_right_up_diag(map)
##print(" move_right_up_diag map",map)

def move_right_down_diag(map):
    i,j=np.where(map ==0)
    if j == 0:
        return None
    else:
        temp_arr = np.copy(map)
        temp = temp_arr[i+1, j+1]
        temp_arr[i,j] = temp
        temp_arr[i+1, j+1] = 0
        if check_location(temp_arr)== True:
            return None
        else:
            return temp_arr
##map=move_right_down_diag(map)
##print(" move_right_down_diag map",map)

####
def get_neighbours(action,map):
    if action == 'up':
##        print(map)
        return move_up(map)
    if action == 'down':
##        print(map)
        return move_down(map)
    if action == 'left':
##        print(map)
        return move_left(map)
    if action == 'right':
##        print(map)
        return move_right(map)
    if action == 'down_right':
##        print(map)
        return move_right_down_diag(map)
    if action == 'up_right':
##        print(map)
        return move_right_up_diag(map)
    if action == "down_left":
##        print(map)
        return move_left_down_diag(map)
    if action == "up_left":
##        print(map)
        return move_left_up_diag(map)        
    else:
        return None
##action = ["down", "up", "left", "right","down_right", "up_right", "down_left", "up_left"]
##for move in action:
##    a=get_neighbours(move,map)
##    print(move,a)

def get_distance(map1,map2):
    p1 = []
    p1.extend(get_robcoord(map1))
##    print(p1,type(p1))
    p2=[]
    p2.extend(get_robcoord(map2))
##    print(p2,type(p2))
    distance = math.sqrt( ((p1[0]-p2[0])**2)+((p1[1]-p2[1])**2) )
    return distance

##a=get_distance(map1,map2)
##print(a)
##def cost_generator(map):
    

def exploring_nodes(node):
    print("Exploring Nodes")
    actions = ["down", "up", "left", "right","down_right", "up_right", "down_left", "up_left"]
    goal_node = map2
    node_q = [node]
    final_nodes = []
    visited = []
    node_cost_list=[] #13
    
    final_nodes.append(node_q[0].map.tolist())  # Only writing data of nodes in seen
    node_counter = 0  # To define a unique ID to all the nodes formed

##    for i in range(2):#while node_q:  # UNCOMMENT FOR DEBUGGING 
    while node_q:
        current_root = node_q.pop(0)  # Pop the element 0 from the list
        if current_root.map.tolist() == goal_node.tolist():
            print("Goal reached",current_root.map,current_root.node_no)
            return child_node, final_nodes, visited
        print("THE LENGTH OF NODE Q at beggining IS: ", len(node_q))

        for move in actions:
            temp_data = get_neighbours(move, current_root.map)
            print(move, temp_data)
            if temp_data is not None:
                node_counter += 1
                print("node count",node_counter)
                node_cost=20000000000000#8b
                move_cost=get_distance(temp_data,current_root.map)
                print("move cost", move_cost)
                cost_to_come= current_root.cost + move_cost#8c
##                print("cost to come", cost_to_comeself.map = map
                child_node = Node(node_counter, np.array(temp_data), current_root, move, node_cost)  # 9
##                print("node_counter", node_counter,"child node point",child_node.map,"parent",child_node.parent,"move",child_node.act,"cost",child_node.cost)
                if node_cost > cost_to_come : #9a
                    child_node.cost= cost_to_come# 9b
                    Dict1={child_node.cost:child_node}
                    for k in Dict1.keys():
                        pass#print("k",k)
                    for i in Dict1.values():
                        pass#print("i",i)
##                print("node_counter", node_counter,"child node point",child_node.map,"parent",child_node.parent,"move",child_node.act,"cost",child_node.cost)

                if child_node.map.tolist() not in final_nodes:  # 10
                    node_q.append(child_node)#12
                    print("THE LENGTH OF NODE Q after children IS: ", len(node_q))
                    final_nodes.append(child_node.map.tolist())#10a
##                    print(final_nodes)
                    visited.append(child_node)#11
                    node_cost_list.append(child_node.cost)
                    min_cost=min(node_cost_list)
                    for k in Dict1.keys():
                        if k==min_cost:
                            print("Dict1[k]",Dict1[k])
                            node_q.remove(Dict1[k])
                            node_q.insert(0,Dict1[k])
                    print("THE LENGTH OF NODE Q after shuffling should be the same as after children IS: ", len(node_q))                   
    return None, None, None  # return statement if the goal node is not reached

def path(node):  # To find the path from the goal node to the starting node
    p = []  # Empty list
    p.append(node)
    parent_node = node.parent
    while parent_node is not None:
        p.append(parent_node)
        parent_node = parent_node.parent
    return list(reversed(p))


"""
get shortes distance from start vortes to each other vortes
get the previous vertex
3 lists
    1. List to keep track of vetexes we have visited
    2. List of vertexes we haven't visited
    3. Distance between visited nodes and start node
Set the value for the distance from start node to start node as zero
set the value for the distance to all other nodes as 20000
START
Visit start node
    Save all locations from all 8 possible moves in List 2
    Get distance from all locations in List 2 to Start node and save in list3
    Add start node to visited list (list 1)
Check to see which node from list 2 has the smallest distance in list3
Visit vertex closest to start node
    Save all locations from all 8 possible moves in List 2
    Get distance from all new locations to current location
    Add distance form above to distance from current location to start location
        if the value of the (distance of current node to start node) + (distance of current nOde to new node) < value for new node and save in list3
    Add current node to visited list (list 1)
    REMOVE IT FROM LIST 2
REPEAT UNTIL NO MORE IN LIST 2"""





def print_states(list_final):  # To print the final states on the console
    print("printing final solution")
    for l in list_final:
        print("Move : " + str(l.act) + "\n" + "Result : " + "\n" + str(l.map) + "\t" + "node number:" + str(l.node_no))
    

start_node=Node(0,map1,None,None,0)
goal, s, v = exploring_nodes(start_node)
print_states(path(goal))
##print(move_left(start_node.node_loc))

