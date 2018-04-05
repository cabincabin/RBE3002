# !/usr/bin/env python
# Clayton Dembski, Floris van Rossum
# RBE3002 - Team 3
# Lab 3 Code

import rospy, tf, copy, math

from geometry_msgs.msg import Twist, Pose, Point, PoseStamped
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
        notEvaluated.put((self.startNode.fCost, self.startNode))

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
        previousSteps = {}

        gCost = {} # Dictionary of Gcosts
        fCost = {} # Dictionary of F Cost
        gCost[self.startNode] = 0 # Cost of travelling from start node to start node is 0

        #fCostGraph[startNode] = self.heuristicCost(startNode) # Calculate the heuristic cost

        # evaluating = []

        pathFound = False # Toggle variable to end loop

        print("---------------- PATHFINDING STARTED -------------")
        print("\nGOING FROM: " + str(self.startNode.point))
        print("\nGOING TO: " + str(self.endNode.point) + "\n\n")

        while not notEvaluated.empty() and not pathFound:
            current = notEvaluated.get()[1]
            current.calculateGCost(self.startNode)
            current.calculateHCost(self.endNode)
            current.calculateFCost()

            #print("Node pulled: X: " + str(current.point.x) + " Y: " + str(current.point.y) + " FCost: " + str(current.fCost))

            if current.isSame(self.endNode):
                # find a way to retrace the path here
                print("--------------A Star Loop Ended-------------")
                print("Path is found")
                pathFound = True
                #previousSteps.append(current)
                print("This is the path: " + str(previousSteps))
                return previousSteps
                continue

            evaluated.append((current))

            # Check the surrounding neighbors of current

            for neighbor in current.connectedNodes:

                print("\t\tThis is the current neighbor under investigation: X: " + str(neighbor.point.x) + " Y: " + str(neighbor.point.y) + " F: " + str(neighbor.fCost))

                if neighbor in evaluated:
                    print("\t\t\tNeighbor is already evaluated")
                    continue

                tentative_gCost = gCost[current] + current.calculateMDistance(neighbor)
                #print("\n\n\n" + str(tentative_gCost) + "\n\n\n")
                # Neighbor is not yet in our discovered set, add it
                if not self.checkIfInArray(notEvaluated, neighbor):
                    gCost[neighbor] = gCost[current] + neighbor.calculateMDistance(current)
                    fCost[neighbor] = gCost[neighbor] + self.heuristicCost(neighbor)
                    notEvaluated.put((fCost[neighbor], neighbor))

                elif (tentative_gCost >= gCost[neighbor]):
                    print("\t\t\tNot a good path")
                    # This is NOT a better path
                    continue

                print("\t\t\tGood path")
                previousSteps[neighbor] = current
                gCost[neighbor] = tentative_gCost
                fCost[neighbor] = abs(gCost[neighbor]) + abs(self.heuristicCost(neighbor))
                print("Updated G: " + str(gCost[neighbor]) + " H: " + str(self.heuristicCost(neighbor)) + " F: " + str(fCost[neighbor]))
        if not pathFound:
            print("--------------A Star Loop Ended-------------")
            print("We have not found a path")
            #previousSteps[self.endNode] = current
            return previousSteps

    def heuristicCost(self, wayPoint):
        return math.sqrt((self.endNode.point.x - wayPoint.point.x) ** 2 + (self.endNode.point.y - wayPoint.point.y) ** 2)

    def checkIfInArray(self, queue, wayPoint):
        while not queue.empty():
            itemGet = queue.get()
            if(wayPoint.isSame(itemGet[1])):
                return True
        else:
             return False

class WayPoint:

    def __init__(self, x, y):
        self.point = Point()
        self.point.x = x
        self.point.y = y
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
        self.hCost = math.sqrt((endNode.point.x - self.point.x)**2 + (endNode.point.y - self.point.y)**2)

    def calculateGCost(self, startNode):
        # Historical cost
        self.gCost = abs(self.point.x - startNode.point.x) + abs(self.point.y - startNode.point.y)

    def calculateFCost(self):
        self.fCost = self.gCost + self.hCost

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
