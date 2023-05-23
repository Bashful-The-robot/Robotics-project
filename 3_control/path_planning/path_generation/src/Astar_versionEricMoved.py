#!/usr/bin/env python3

import math
import numpy as np
import ros_numpy


class Node:
    def __init__(self, x,y,parent=None):
        self.x = x
        self.y = y
        self.g_cost = 0             #G cost: Cost from start node to the goal node
        self.h_cost = 0             #H cost: Distance from end node (heuristic cost)
        self.parent = parent

    def f_cost(self):
        return self.g_cost+self.h_cost
    
class Astar:
    def __init__(self,grid_np,grid_data):
        self.grid = grid_np
        self.height = grid_np.shape[1]  
        self.width = grid_np.shape[0]   

        self.resolution = grid_data.info.resolution
        self.xmin = grid_data.info.origin.position.x
        self.ymin = grid_data.info.origin.position.y
        self.xmax = 4.42
        self.ymax = 9.79

    
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

                if cond_x and cond_y and self.grid[x,y] !=100:  #Inside the gridmap and not occupied
                    neighbors[Node(x,y)] = self.grid[x,y]

        return neighbors
    def convert_to_grid(self,pos):
        #Converts the position in map frame into nodes in the gridmap
        x = pos.x
        y = pos.y
        xgrid = int((pos.x -self.xmin) / self.resolution)
        ygrid = int((pos.y - self.ymin) / self.resolution)
        return Node(xgrid,ygrid)

    def get_trajectory(self,start,goal):
        start = self.convert_to_grid(start)
        #goal = self.convert_to_grid(goal)
        open_list = [start]
        closed_list = []

        try:
            if self.grid[goal.x,goal.y] == 100:                             #Goal is on obstacles       
                print("Unvalid goal position")
                return None

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
                        return trajectory
                    
                    for neighbor in self.get_vacant_neighbors(current):
                        if neighbor in closed_list:                         #Do nothing
                            continue
                        if neighbor not in open_list:                       #Add  to list
                            neighbor.g_cost = current.g_cost + 1
                            neighbor.h_cost = self.get_euclidian_dist(neighbor,goal)
                            neighbor.parent = current
                            open_list.append(neighbor)
                        else:
                            if current.g_cost +1 < neighbor.g_cost:  
                                neighbor.g_cost = current.g_cost +1
                                neighbor.parent = current
            return None
        except:
            print("Error occured!")
    
    def get_explorerNode(self):
        x,y = np.where(self.grid == -1)
        if not x.any():        #If occupancy grid doesn't contain -1
            return None
        return Node(x[0],y[0])