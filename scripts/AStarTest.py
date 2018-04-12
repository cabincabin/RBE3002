# !/usr/bin/env python
# Clayton Dembski, Floris van Rossum
# RBE3002 - Team 3
# Purpose: Lab 3 Astar Algorithm Code

#####################IMPORTS##############################
import rospy, tf, copy, math

from geometry_msgs.msg import Twist, Pose, Point, PoseStamped
from tf.transformations import euler_from_quaternion
import math
import Queue as Q

from Draw import drawGrid

import numpy as np
from std_msgs.msg import String
##########################################################

# A star costs, heuristics and functions
# h(n) = Manhattan distance to start
# g(n) = Euclidian distance to goal
# f(n) = g(n) + h(n)

class AStar:
    def __init__(self, startNode, endNode, distance):
        self.startNode = startNode # The starting waypoint
        self.endNode = endNode # The ending waypoint
        self._distance = distance

    # Find the best path with a star
    def findPath(self):
        evaluated = [] # Contains all the nodes that have been evaluated
        notEvaluated = Q.PriorityQueue() # PriorityQueue of nodes based on fCost
        notEvaluated.put((self.startNode.fCost, self.startNode)) # Insert start node

        # This code was used to test the algorithm
        # ##################### TESTING CODE #####################
        # newNode1 = WayPoint(20,130)
        # newNode2 = WayPoint(110,20)
        # newNode1.calculateGCost(startNode)
        # newNode2.calculateGCost(startNode)
        # newNode1.calculateHCost(endNode)
        # newNode2.calculateHCost(endNode)
        # newNode1.calculateFCost()
        # newNode2.calculateFCost()
        #
        # notEvaluated.put((newNode1.fCost, newNode1))
        # notEvaluated.put((newNode2.fCost, newNode2))
        #
        # startNode.connectedNodes = [newNode1, newNode2]
        # newNode1.connectedNodes = [startNode, newNode2, endNode]
        # endNode.connectedNodes = [newNode1]
        # newNode2.connectedNodes = [newNode1, startNode]
        #
        # ###################### END TESTING CODE #################
        previousSteps = {} # Dictionary of the previous nodes chosen

        gCost = {} # Dictionary of Gcosts (key = node)
        fCost = {} # Dictionary of F Cost, (key = node)
        gCost[self.startNode] = 0 # Cost of travelling from start node to start node is 0

        pathFound = False # Toggle variable to end loop

        # Printing that AStar is starting
        print("---------------- PATHFINDING STARTED -------------")
        print("\nGOING FROM: " + str(self.startNode.point))
        print("\nGOING TO: " + str(self.endNode.point) + "\n\n")

        # AStar Loop
        while not notEvaluated.empty() and not pathFound:
            current = notEvaluated.get()[1] # Pull the best node from the queue based on its fCost

            # Print statement for debugging
            #print("Node pulled: X: " + str(current.point.x) + " Y: " + str(current.point.y) + " FCost: " + str(current.fCost))

            # Check if the current node pulled is our goal
            if current.isSame(self.endNode):
                # We have reached the end node, exit the loop and return path
                print("--------------A Star Loop Ended-------------")
                print("Path is found")
                pathFound = True
                #print("This is the path: " + str(previousSteps))
                return previousSteps
                continue

            # Add current to the evalated array
            evaluated.append((current))
            drawGrid('/nav_msgs/GridCellsChecked', evaluated, self._distance)

            # Check the surrounding neighbors of current
            for neighbor in current.connectedNodes:
                # Print statment for debugging
                #print("\t\tThis is the current neighbor under investigation: X: " + str(neighbor.point.x) + " Y: " + str(neighbor.point.y) + " F: " + str(neighbor.fCost))

                # Check if the neighbor has been checked before
                if neighbor in evaluated:
                    # Neighbor has already been checked so exit loop
                    # Print for debugging
                    #print("\t\t\tNeighbor is already evaluated")
                    continue

                # Calculate the new GCost with the neighbor
                tentative_gCost = gCost[current] + current.calculateMDistance(neighbor)

                # Print statement for debugging
                #print("\n\n\n" + str(tentative_gCost) + "\n\n\n")

                # Check if neighbor has been discovered yet
                if not self.checkIfInArray(notEvaluated, neighbor):
                    # Neighbor is not yet in our discovered set, add it to PriorityQueue
                    gCost[neighbor] = gCost[current] + neighbor.calculateMDistance(current)
                    fCost[neighbor] = gCost[neighbor] + self.heuristicCost(neighbor)
                    notEvaluated.put((fCost[neighbor], neighbor))

                # Check if this is a good path
                elif (tentative_gCost >= gCost[neighbor]):
                    # tentative_gCost is higher than gCost[neighbor] so this is a bad path
                    #print("\t\t\tNot a good path")
                    continue

                #print("\t\t\tGood path")
                # Good path choice, update the values and costs, add the node to previousSteps path
                previousSteps[neighbor] = current
                gCost[neighbor] = tentative_gCost
                fCost[neighbor] = abs(gCost[neighbor]) + abs(self.heuristicCost(neighbor))
                # Print statement for debugging
                #print("Updated G: " + str(gCost[neighbor]) + " H: " + str(self.heuristicCost(neighbor)) + " F: " + str(fCost[neighbor]))

        # If this is reached, no path was found and print an error
        if not pathFound:
            print("--------------A Star Loop Ended-------------")
            print("We have not found a path")
            #previousSteps[self.endNode] = current
            return previousSteps

    # Calculate the heuristics cost with a certain waypoint
    def heuristicCost(self, wayPoint):
        return math.sqrt((self.endNode.point.x - wayPoint.point.x) ** 2 + (self.endNode.point.y - wayPoint.point.y) ** 2)

    # Method to check if a node is contained in the PriorityQueue
    # Copies the PriorityQueue to an array, then reconstructs the PriorityQueue
    def checkIfInArray(self, queue, wayPoint):
        listQ = []
        listQ2 = []
        while not queue.empty():
            itemGet = queue.get()
            listQ.append(itemGet)
            listQ2.append(itemGet[1])
            if wayPoint == itemGet[1] or (wayPoint.point.x == itemGet[1].point.x and wayPoint.point.y == itemGet[1].point.y):
                # We have found the node we were looking for
                for i in range(len(listQ)):
                    queue.put(listQ[i])
                return True
        # The node we were looking is not found
        drawGrid('nav_msgs/GridCellsFrontier',listQ2, self._distance)
        for i in range(len(listQ)):
            queue.put(listQ[i])
        return False

# Waypoint (node) class
# Purpose: A node for a graph, called wayPoint to avoid confusion
class WayPoint:

    def __init__(self, x, y):
        self.point = Point()
        self.point.x = x
        self.point.y = y
        # 100 for wall
        # 0 for empty
        # -1 for unknown
        self.fCost = 0 # Total cost

        # Array containing all the connected nodes to this node
        self.connectedNodes = []

    # Check if this node and compareNode are occupying the same space
    def isSame(self, compareNode):
        if((compareNode.point.x == self.point.x) and (compareNode.point.y == self.point.y)):
            return True
        else:
            return False

    # Return the Manhattan distance between this node and a compared node
    def calculateMDistance(self, compareNode):
        return (abs(self.point.x - compareNode.point.x) + abs(self.point.y - compareNode.point.y))

if __name__ == '__main__':
    endNode = WayPoint(1000,1000)
    startNode = WayPoint(0,0)
    aStar = AStar(startNode,endNode)
    aStar.findPath()
