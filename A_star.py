from Map import *
import math


class Node():
    def __init__(self, state, g, h, status, parent, kids):
        self.state = state #a state representing the position of the node
        self.g = g #representing the "distance" from start to curr node (q)
        self.h = h #estimate of distance from current node (q) to goal state
        self.f = g + h
        self.status = status #0 for closed, 1 for open
        self.parent = parent
        self.kids = kids



def h_cost(curr_state): 
    #Euclidian heuristic, distance between current state and goal state
    return math.sqrt((curr_state[0]- goal_state[0])**2 + (curr_state[1] - goal_state[1])**2)

def pop_smallest_f_open_nodes():
    #pops the node with the smallest f value from the open_node list
    f_min = 100000
    node_min = open_nodes[0]

    for node in open_nodes:
        if node.f < f_min:
            f_min = node.f
            node_min = node
    
    open_nodes.remove(node_min)
    return node_min


def A_star():
    h = h_cost(init_state)
    n0 = Node(init_state, 0, h, 1, None, None) #status = 1, meaning open
    open_nodes.append(n0)

    #Agenda loop
    while (open_nodes != []):
        Q = pop_smallest_f_open_nodes() #curr_node(Q) = the node from open_nodes with the lowest f value
        successor_nodes = generate_succesors(Q) #open all neighbours of current node
        
        for S in successor_nodes: #go through all successors of the current node
            
            if S.state == goal_state:
                print("FOUND SOLUTION!")
                return S #returns goal node
            
            #update the successor's f value
            S.g = Q.g + map_obj.get_cell_value(S.state) # map_obj.get_cell_value(state) is 'cost' between sucessor and parent
            S.h = h_cost(S.state)
            S.f = S.g + S.h

            if in_open_AND_f_lower(S): #ii)
                continue; #skips successor

            if in_closed_AND_f_lower(S): #iii)
                continue; #skips successor

        closed_nodes.append(Q) #order of closed_nodes dont matter
    #end while

def in_open_AND_f_lower(S): 
    #S is a successor node of Q

    for node in open_nodes:
        #if there is a an opened node with the same postion as S, which also has a lower f value than S
        if (node.state == S.state and node.f < S.f):
            return True #return True (meaning sucessor S will be skipped)
    return False

def in_closed_AND_f_lower(S):
    #S is a successor node of Q
    for node in closed_nodes:
        #if there is a closed node with the same postion as S, which also has a lower f value than S
        if (node.state == S.state and node.f < S.f): 
            return True; #return True (meaning sucessor S will be skipped)
    
    #otherwise open the node
    open_nodes.append(S)
    return False


def generate_succesors(X):
    #opens all nodes around node X

    succ_list = []

    parent = X
    status = 1 # meaning open
    kids = None

    #open node below
    state = [parent.state[0] + 1, parent.state[1]] #y, x
    if (map_obj.get_cell_value(state) != -1): #there is not a wall 
        h = h_cost(state)
        g = parent.g + map_obj.get_cell_value(state)
        succ_list.append(Node(state,g,h,status,parent,kids))

    #open node over
    state = [parent.state[0] - 1, parent.state[1]] #y, x
    if (map_obj.get_cell_value(state) != -1): #there is not a wall 
        h = h_cost(state)
        g = parent.g + map_obj.get_cell_value(state)
        succ_list.append(Node(state,g,h,status,parent,kids))
    
    #open node to the right
    state = [parent.state[0], parent.state[1] + 1] #y, x
    if (map_obj.get_cell_value(state) != -1): #there is not a wall 
        h = h_cost(state)
        g = parent.g + map_obj.get_cell_value(state)
        succ_list.append(Node(state,g,h,status,parent,kids))
    
    #open node to the left
    state = [parent.state[0], parent.state[1] - 1] #y, x
    if (map_obj.get_cell_value(state) != -1): #there is not a wall 
        h = h_cost(state)
        g = parent.g + map_obj.get_cell_value(state)
        succ_list.append(Node(state,g,h,status,parent,kids))

    return succ_list #return the sucessors

#MAIN
#create map
map_obj = Map_Obj(4)
closed_nodes=[] #list over all the nodes that havent been fully explored yet.
open_nodes=[] #list over nodes that potentially could lead to the optimal path, arent fully explored yet. 

init_state = [map_obj.start_pos[0], map_obj.start_pos[1]] #map_obj is saved as (y,x)
goal_state = [map_obj.goal_pos[0], map_obj.goal_pos[1]]

goal_node = A_star()

#Draw path
node = goal_node
map_obj.set_cell_value(node.state, ' Y ')
while(node.parent != None):
    node = node.parent
    map_obj.set_cell_value(node.state, ' Y ')

map_obj.show_map()
#END MAIN



#PSEUDOCODE
#https://www.geeksforgeeks.org/a-search-algorithm/