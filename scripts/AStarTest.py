# !/usr/bin/env python
# Clayton Dembski, Floris van Rossum
# RBE3002 - Team 3
# Lab 3 Code

import rospy, tf, copy, math

from geometry_msgs.msg import Twist, Pose, PoseStamped
from tf.transformations import euler_from_quaternion
import math
import Queue as Q

import numpy as np
from std_msgs.msg import String


# h(n) = Manhattan distance to start
# g(n) = Euclidian distance to goal

class AStar:
    def __init__(self, startNode, endNode):
        # Put initialization here
        self.startNode = startNode # The starting waypoint
        self.endNode = endNode # The ending waypoint

    # Find the best path
    def findPath(self):
        evaluated = []
        notEvaluated = Q.PriorityQueue()
        notEvaluated.put((startNode.fCost, startNode))

        ##################### TESTING CODE #####################
        newNode1 = WayPoint(20,130)
        newNode2 = WayPoint(110,20)
        newNode1.calculateGCost(startNode)
        newNode2.calculateGCost(startNode)
        newNode1.calculateHCost(endNode)
        newNode2.calculateHCost(endNode)
        newNode1.calculateFCost()
        newNode2.calculateFCost()

        notEvaluated.put((newNode1.fCost, newNode1))
        notEvaluated.put((newNode2.fCost, newNode2))

        startNode.connectedNodes = [newNode1, newNode2]
        newNode1.connectedNodes = [startNode, newNode2, endNode]
        endNode.connectedNodes = [newNode1]
        newNode2.connectedNodes = [newNode1, startNode]

        ###################### END TESTING CODE #################
        previousSteps = {}

        gCost = {} # Dictionary of Gcosts
        fCost = {} # Dictionary of F Cost
        gCost[startNode] = 0 # Cost of travelling from start node to start node is 0

        #fCostGraph[startNode] = self.heuristicCost(startNode) # Calculate the heuristic cost

        # evaluating = []

        pathFound = False # Toggle variable to end loop

        while not notEvaluated.empty() and not pathFound:
            current = notEvaluated.get()[1]
            current.calculateGCost(startNode)
            current.calculateHCost(endNode)
            current.calculateFCost()

            print("Node pulled: X: " + str(current.x) + " Y: " + str(current.y) + " FCost: " + str(current.fCost))

            if current.isSame(endNode):
                # find a way to retrace the path here
                # return previousSteps + Current as a path
                print("--------------A Star Loop Ended-------------")
                print("Path is found")
                pathFound = True
                continue


            evaluated.append((current))

            # Check the surrounding neighbors of current

            for neighbor in current.connectedNodes:

                print("\t\tThis is the current neighbor under investigation: X: " + str(neighbor.x) + " Y: " + str(neighbor.y) + " F: " + str(neighbor.fCost))

                if neighbor in evaluated:
                    print("\t\t\tNeighbor is already evaluated")
                    continue

                tentative_gCost = gCost[current] + current.calculateMDistance(neighbor)

                try:
                    gCost[neighbor]
                except KeyError:
                    gCost[neighbor] = current.calculateMDistance(neighbor)
                # Neighbor is not yet in our discovered set, add it
                if not self.checkIfInArray(notEvaluated, neighbor):
                    notEvaluated.put((neighbor.fCost, neighbor))

                elif (tentative_gCost >= gCost[neighbor]):
                    # This is NOT a better path
                    continue

                previousSteps[neighbor] = current
                gCost[neighbor] = tentative_gCost
                fCost[neighbor] = gCost[neighbor] + self.heuristicCost(neighbor)

        if not pathFound:
            print("--------------A Star Loop Ended-------------")
            print("We have not found a path")

    def heuristicCost(self, wayPoint):
        return math.sqrt((endNode.x - wayPoint.x) ** 2 + (endNode.y - wayPoint.y) ** 2)

    def checkIfInArray(self, queue, wayPoint):
        while not queue.empty():
            itemGet = queue.get()
            if(wayPoint.isSame(itemGet[1])):
                return True
        else:
             return False

class WayPoint:

    def __init__(self, x, y):
        self.x = x
        self.y = y
        self.hCost = 0 # Euclidian distance (future cost)
        # 100 for wall
        # 0 for empty
        # -1 for unknown
        self.cost = 0
        self.gCost = 0 # Historical cost
        self.fCost = 0 # Total cost

        # Array containing all the connected nodes to this node
        self.connectedNodes = []

    # The waypoint's h cost
    def calculateHCost(self, endNode):
        # The Manhattan Distance to the beginning
        self.hCost = math.sqrt((endNode.x - self.x)**2 + (endNode.y - self.y)**2)

    def calculateGCost(self, startNode):
        # Historical cost
        self.gCost = (self.x - startNode.x) + (self.y - startNode.y)

    def calculateFCost(self):
        self.fCost = self.gCost + self.hCost

    # Check if this node and compareNode are occupying the same space
    def isSame(self, compareNode):
        if((compareNode.x == self.x) and (compareNode.y == self.y)):
            return True
        else:
            return False

    # Return the Manhattan distance between this node and a compared node
    def calculateMDistance(self, compareNode):
        return (abs(self.x - compareNode.x) + abs(self.y - compareNode.y))

if __name__ == '__main__':
    endNode = WayPoint(1000,1000)
    startNode = WayPoint(0,0)
    aStar = AStar(startNode,endNode)
    aStar.findPath()
