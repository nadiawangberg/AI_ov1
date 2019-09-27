from Map import *
import math


class Node():
    def __init__(self, state, g, h, status, parent, kids):
        self.state = state
        self.g = g
        self.h = h
        self.f = g + h
        self.status = status #0 for closed, 1 for open
        self.parent = parent
        self.kids = kids



def h_cost(curr_state): #works
    return math.sqrt((curr_state[0]- goal_state[0])**2 + (curr_state[1] - goal_state[1])**2)


#print(init_state[0]) #18 --> x 
#print(init_state[1]) #27 --> y

def pop_smallest_f_open_nodes():
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
    #print("h: ", n0.f)

    #Agenda loop
    while (open_nodes != []):
        #print("len_open:", len(open_nodes))
        #pop the smallest element from open_nodes
        Q = pop_smallest_f_open_nodes()
        successor_nodes = generate_succesors(Q) #checked ish
        print("len_succ: ", len(successor_nodes))
        for S in successor_nodes:
            print("len_open:", len(open_nodes))
            print("len_closed:", len(closed_nodes))

            if S.state == goal_state:
                print("FOUND SOLUTION!")
                print("S.state: ", S.state)
                return S #returns goal node
            
            S.g = Q.g + 1 # 1 is dist between sucessor and parent
            S.h = h_cost(S.state)
            S.f = S.g + S.h


            if in_open_AND_f_lower(S): #ii)
                continue; #skip successor

            if in_closed_AND_f_lower(S): #iii)
                continue; #skip successor

        closed_nodes.append(Q) #order of closed_nodes dont matter
    #end while

def in_open_AND_f_lower(S):
    for node in open_nodes:
        if (node.state == S.state and node.f < S.f): #if same pos
            return True
    return False

def in_closed_AND_f_lower(S):
    print("in func")
    for node in closed_nodes:
        if (node.state == S.state and node.f < S.f):
            print("if")
            return True;

    print("no nodes in closed with same state and lower f")
    open_nodes.append(S)
    #TODO  maybe i also need to delete a node?
    return False




#print("GOAL VALUE!!!", map_obj.get_cell_value([40, 32])) # 1 is not wall -1 is wall


def generate_succesors(X):
    succ_list = []

    #for all points
    parent = X
    status = 1 # meaning open
    kids = None

    #down
    #print("parent state: ", parent.state)
    state = [parent.state[0] + 1, parent.state[1]] #y, x
    if (map_obj.get_cell_value(state) != -1): #there is not a wall 
        h = h_cost(state)
        g = parent.g + 1
        #should be sorted??
        succ_list.append(Node(state,g,h,status,parent,kids))

    #up
    state = [parent.state[0] - 1, parent.state[1]] #y, x
    if (map_obj.get_cell_value(state) != -1): #there is not a wall 
        h = h_cost(state)
        g = parent.g + 1
        #should be sorted??
        succ_list.append(Node(state,g,h,status,parent,kids))
    
    #right
    state = [parent.state[0], parent.state[1] + 1] #y, x
    if (map_obj.get_cell_value(state) != -1): #there is not a wall 
        h = h_cost(state)
        g = parent.g + 1
        #should be sorted??
        succ_list.append(Node(state,g,h,status,parent,kids))
    
    #left
    state = [parent.state[0], parent.state[1] - 1] #y, x
    if (map_obj.get_cell_value(state) != -1): #there is not a wall 
        h = h_cost(state)
        g = parent.g + 1
        #should be sorted??
        succ_list.append(Node(state,g,h,status,parent,kids))

    return succ_list

#MAIN

#create map
map_obj = Map_Obj(2)

closed_nodes=[]#empty node list 
open_nodes=[] #empty node list

init_state = [map_obj.start_pos[0], map_obj.start_pos[1]] #map_obj is saved as (y,x)
goal_state = [map_obj.goal_pos[0], map_obj.goal_pos[1]]


goal_node = A_star()

node = goal_node
map_obj.set_cell_value(node.state, ' Y ')
while(node.parent != None):
    node = node.parent
    map_obj.set_cell_value(node.state, ' Y ')
    #tegne!!


map_obj.show_map()

#END MAIN









            #double_break = False
            #for node in open_nodes: #ii)
            #    if (node.state == S.state and node.f < S.f): #if same pos
            #        double_break = True
            #        break;
            #if double_break:
            #    double_break = False
            #    break;

            #for node in closed_nodes: #iii)
            #    if (node.state == S.state and node.f < S.f):
            #        double_break = True
            #        break;
            #    else:
            #        open_nodes.append(node)
            #if double_break:
            #    double_break = False
            #    break;
