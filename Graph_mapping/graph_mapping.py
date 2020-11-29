from collections import defaultdict
import math
from shortest_path import dijsktra

class Graph():
    def __init__(self):
        
        """
        self.edges is a dict of all possible next nodes
        e.g. {'X': ['A', 'B', 'C', 'E'], ...}
        self.weights has all the weights between two nodes,
        with the two nodes as a tuple as the key
        e.g. {('X', 'A'): 7, ('X', 'B'): 2, ...}
        """
        
        self.edges = defaultdict(list)
        self.weights = {}                                   # this is the distance between two nodes
        self.co_ordinates = [(0,0) for i in range(15)]     # this is the odom values of a node


    def add(self, from_node, to_node, odom_x, odom_y):      # odom_x = odom_from = (a,b)
        
        '''
        Note: assumes edges are bi-directional
        odom_from = (1,2)
        odom_to = (1,3) i.e in a tuple format containing (x,y) cartesian co-ordinate
        '''

        if to_node not in self.edges[from_node]: 
            self.edges[from_node].append(to_node)
            self.edges[to_node].append(from_node)

        self.co_ordinates[from_node]=odom_x
        self.co_ordinates[to_node]=odom_y

        weight = (odom_x[0]-odom_y[0])**2 + (odom_x[1]-odom_y[1])**2
        weight = math.sqrt(weight)
        
        self.weights[(from_node, to_node)] = weight
        self.weights[(to_node, from_node)] = weight               

    def check_repeated_node(self,odom_x): 

        '''
        Func to check if the node in a graph is repeated, if not create a new node
        
        Parameter : co-ordinates of the current node
        Return value : current node position of the bot

        '''

        threshold = 1    #Error allowed 
        counter=0        # To store the node number
        flag = 0         # 0 means create new node, 1 means exisiting node found
        for odom_y in self.co_ordinates:
            diff = (odom_x[0]-odom_y[0])**2 + (odom_x[1]-odom_y[1])**2
            if diff < threshold:
                node_ref = counter
                flag=1
                break
            counter+=1
        
        if flag==0:
            node_ref = -1    #this means refer the node encountered

        return node_ref       


    def print_graph(self):
        print('-----------------')
        print('Edges = ',self.edges)
        print('Distances = ',self.weights)
        print('Co-ordinates = ',self.co_ordinates)
        print('-----------------')

    def shortest_path(self,path):

        '''
        Func to convert the shortest node path to shortest cartesian path that the urdf bot has to follow

        parameter : shortest path corresponding to nodes
        return value : shortest path corresponding to cartesian co-ordinates

        '''

        co_ordinate_path = [self.co_ordinates[i] for i in path]
        return co_ordinate_path

g = Graph()

more_than_2_exits=True
took_a_turn=True
dead_end=True

start_node=1
prev_node=start_node       # References thoroughout the code
curr_node=2

prev_odom = (0,0)


# Set of hardcoded values. Input these values continuously from urdf model in the run 

nodes=2
odom = [(3,0),(3,3),(10,3),(3,3.5),(3,0.5),(7,0),(7,-2),(9,-2)]


for i in range(len(odom)):
    curr_node = nodes
    curr_odom = odom[i]
    
    if more_than_2_exits or took_a_turn or dead_end:

        node_ref = g.check_repeated_node(curr_odom)
        if node_ref != -1 :
            curr_node = node_ref  
        else:
            nodes+=1  
        g.add(prev_node,curr_node,prev_odom,curr_odom)

    prev_node = curr_node
    prev_odom = curr_odom
    g.print_graph()

end_node = curr_node

node_shortest_path = dijsktra(g,start_node,end_node)
co_orindate_shortest_path = g.shortest_path(node_shortest_path)

print('The ideal shortest node path to follow = ',node_shortest_path)
print('The ideal shortest co-ordinate path to follow = ',co_orindate_shortest_path)