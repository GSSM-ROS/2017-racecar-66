#Imports
import numpy
import cv2
import Queue
import math
#import matplotlib.pyplot as plt
import time
import csv

sqrt2 = math.sqrt(2)


def findPath(m, (origin_x, origin_y), (goal_x, goal_y), freespace = 0):
    '''
    Finds the path using an A* algorithm
    
    :param filename: name of file for use of pathfinding
    :param origin: a tuple with the X and Y coordinates of the origin
    :param goal: a tuple with the X and Y coordinates of the goal
    :param freespace: optional param for an array of acceptable values
    
    :return current, parents, costs[current]: returns a list of the parents, the current(goal achieved) point, ans its cost
    :return parents, costs: returns the explored area and its costs if no goal found
    '''

    
    start = (origin_x, origin_y)            #sets origin, goal points
    goal = (goal_x, goal_y)

    frontier = Queue.PriorityQueue()        #uses prioirty queue to prioritize points closer to the goal
    frontier.put((0, start))

    parents = {}                            #list of parents for traceback of goalpoint
    parents[start] = None

    costs = {}                              #list of costs to keep track of previous locations
    costs[start] = 0
    
    
    while not frontier.empty():             #loop through frontier
        current = frontier.get()[1]         #acquire a block

        if current == goal :                #check if its the goal
            print("\n Path found ! ")
            return current, parents, costs[current]

        if m[current[0]][current[1]] != freespace:
            continue
        
        #print(current)
        #print(costs[current])
 
        for next in neighbors(current):     #iterate through the neighbors of a point

            
            this_cost = costs[current] + dist(current, next) #calculate cost for minimal distance
            
            

            
            if not next in costs or this_cost < costs[next]: #if neighbor is new / has new low cost
                
                costs[next] = this_cost
                
                p = this_cost + heur(next, goal)
                
                frontier.put((p, next))
                parents[next] = current
               


    return parents, costs, -1

    


def neighbors(point):
    '''
    returns the eight nearby points of one point
    
    :param point: the point to be analyzed
    
    :return result: a tuple of the neighbors 
    '''
    x,y = point
    result =   [ (x-1 , y-1) , (x , y-1) , (x+1 , y-1) ,      #all nearby indicies
                 (x-1 , y  ) ,             (x+1 , y  ) ,
                 (x-1 , y+1) , (x , y+1) , (x+1 , y+1) ]
    
    #OLD
    #for near in neighbors:                          #iterate through indecies list
    #    result.append(( point[0] + near[0] , point[1] + near[1] ))      #Add all indecies to point, append
    return result

def heur(end, check):
    '''
    returns the goemetric distance between two points using diagonal and grid lines for distance
    
    :param end: the end point to be compared
    :param check: the point to be checked
    
    :return distance: the distance between the two points
    '''
    #old manhattan
    #result = math.fabs(end[0] - check[0]) + math.fabs(end[1] - check[1])
    #old pythagoran
    #result = math.sqrt((end[0] - check[0])**2 + (end[1] - check[1]) **2) 
    
    #new - from Hunter cause its smart
    dx = abs(end[0] - check[0])
    dy = abs(end[1] - check[1])

    result = min(dx, dy) * sqrt2 + abs(dx-dy)     

    return result


def dist(start, end):
    '''
    returns the distance to a neighbor
    
    :param start: the original point
    :param end: the point to be measured to
    
    :return distance: returns the neighboring distance
    '''

    
    if start[0] == end[0] or start[1] == end[1]: #if not on the same line
        return 1
    else: 
        return sqrt2


def aStar(start, goal, occupancyGrid, free = 255):
    win, parents, cost = findPath(occupancyGrid, start, goal, freespace = free)
    if cost == -1:
        return (start)
    else:
        path = []
        trace = win
        while trace != None:
            path.append(trace)
            trace = parents[trace]
        return path

