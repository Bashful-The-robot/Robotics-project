#!/usr/bin/env python3

import matplotlib.pyplot as plt

import math
import numpy as np
import ros_numpy
from shapely.geometry import Point
from shapely.geometry.polygon import Polygon


class Node:
    def __init__(self, x,y,parent=None):
        self.x = x
        self.y = y
        self.g_cost = 0             #G cost: Cost from start node to the goal node
        #self.cell_cost = 0
        self.h_cost = 0             #H cost: Distance from end node (heuristic cost)
        self.parent = parent

    def f_cost(self):
        return self.g_cost+self.h_cost
    
class Astar:
    # def __init__(self,grid_np):
    #     self.grid = grid_np                            #The entire grid
    #     self.height = grid_np.shape[1]  
    #     self.width = grid_np.shape[0]   

    def __init__(self,grid,grid_data,geofence):
        self.grid = grid                            #The entire grid
        self.height = grid.shape[0] 
        self.width = grid.shape[1]


        self.resolution = grid_data[0]
        self.xmin = grid_data[1]
        self.ymin = grid_data[2]


        self.poly = Polygon(geofence)                   #The workspace we should stay within

    ###########changing now - dont forget to change self.grid[x,y] to self.grid[y,x] on line 56
    def get_euclidian_dist(self,node1,node2):
        return math.sqrt( (node1.x-node2.x)**2 + (node1.y-node2.y)**2 )

    def get_vacant_neighbors(self,node):
        neighbors = {}
        for i in range(-1,2):                                   #Loops through x-coordinates of grid
            for k in range(-1,2):       
                x = node.x + i
                y = node.y + k
                cond_x = (x >= 0 and x < self.width)            #Inside gridmap (origin in (0,0))
                cond_y = (y >=0 and y < self.height)    

                if cond_x and cond_y and self.grid[x,y] <= 60:  #Inside the gridmap and not occupied
                    neighbors[Node(x,y)] = self.grid[x,y]

        return neighbors
    
    def convert_to_grid(self,pos):
        #Converts the position in map frame into nodes in the gridmap
        x = pos.x
        y = pos.y
        xgrid = int((pos.x -self.xmin) / self.resolution)
        ygrid = int((pos.y - self.ymin) / self.resolution)

        return Node(xgrid,ygrid)

    def convert_to_map(self,pos):
    #Converts the position from grid map into coordinates in the map frame
        x_g = pos.x
        y_g = pos.y
        x_m = x_g*self.resolution+ self.xmin
        y_m = y_g*self.resolution+ self.ymin
        return (x_m, y_m)
    
    def flip(self,start):
        return Node(start.y,start.x)
    
    def get_trajectory(self,start,goal):
        
        print("looking for trajectory")
        start = self.convert_to_grid(start)
        start = self.flip(start)
        print("The start point is ",start.x,start.y)
        print("The goal point is ",goal.x,goal.y)
        open_list = [start]
        closed_list = []
        removeLastElement = False
        #try:
        if self.grid[goal.x,goal.y]== 100:                             #Goal is on obstacles   
            removeLastElement = True      
            self.grid[goal.x,goal.y]= 0
                                              

        while len(open_list) > 0:
            current = min(open_list, key=lambda node: node.f_cost())    #Selects element from list with smallest f cost
            open_list.remove(current)
            if current not in closed_list:
                closed_list.append(current)

                if (current.x == goal.x) and (current.y==goal.y):       #If goal is reached
                    trajectory = []
                    while current is not None:                          #While we've not reached the last node
                        trajectory.append(current)
                        current = current.parent
                    trajectory.reverse()
                    if removeLastElement == True:                       #In order to not run over the obstacle
                        del trajectory[-1] 
                    #self.plot_traj(trajectory)                                 
                    return trajectory
                
                vacant_neighbors = self.get_vacant_neighbors(current)
                if len(vacant_neighbors) == 0:
                    print("No neighbors")
                    return None
                
                for neighbor in vacant_neighbors:
                    if neighbor in closed_list:                         #Do nothing
                        continue
                    if neighbor not in open_list:                       #Add  to list
                        neighbor.g_cost = current.g_cost + 1
                        neighbor.h_cost = self.get_euclidian_dist(neighbor,goal)
                        neighbor.parent = current
                        open_list.append(neighbor)
                        self.grid[current.x,current.y] =100

        print("A* doesn't find a path")
        return None

    def plot_traj(self,traj):
        for node in traj:
            self.grid[node.x,node.y] = 100
        plt.imshow(self.grid, cmap='gray', origin='lower')
        plt.title('New Grid')
        plt.colorbar()
        plt.show()  
        input()

    def isInsideWS(self,points):
        points = self.flip(points)
        #Checks if the point is inside the polygon 
        x_m,y_m = self.convert_to_map(points)
        point = Point(x_m,y_m)
        if self.poly.contains(point):
            return points
        else:
            return None
            
    
    def get_explorerNode(self):
        x, y = np.where(np.isclose(self.grid, -1.0))
        if not x.any():               #If occupancy grid doesn't contain -1, we've done exploring.
            return None
        else:
            return Node(x[0],y[0])      #return first node
        


# grid = np.array([[0,0,0,0],
#                 [100,100,100,0],
#                 [0,0,0,0]])
# #notation: grid[x,y] where x horizontal and y vertical
# print(grid)
# x=0
# y=0
# start = Node(2,3)
# goal = Node(x,y)
# astarobj = Astar(grid)
# path = astarobj.get_trajectory(start,goal)
# if path != None:
#     for node in path:
#         grid[node.x,node.y] = 2
#         print("(x,y): "+ str(node.x) + ", " + str(node.y))

# print(grid)