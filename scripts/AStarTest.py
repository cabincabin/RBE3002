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

        newNode1 = WayPoint(20,130)
        newNode2 = WayPoint(110,20)
        newNode1.calculateGCost(startNode)
        newNode2.calculateGCost(startNode)
        newNode1.calculateHCost(endNode)
        newNode2.calculateHCost(endNode)
        newNode1.calculateFCost()
        newNode2.calculateFCost()

        notEvaluated.put((startNode.fCost, startNode))
        notEvaluated.put((newNode1.fCost, newNode1))
        notEvaluated.put((newNode2.fCost, newNode2))
        previousSteps = []

        gCostGraph = [] #map with default value 100 # TODO What is this?

        #gCostGraph[startNode] = 0 # Cost of travelling from start node to start node is 0

        #fCostGraph[startNode] = self.heuristicCost(startNode) # Calculate the heuristic cost

        # evaluating = []

        while not notEvaluated.empty():
            current = notEvaluated.get()[1]
            print("Node pulled: X: " + str(current.x) + " Y: " + str(current.y) + " FCost: " + str(current.fCost))

            if current.isSame(endNode):
                # find a way to retrace the path here

        print("--------------A Star Loop Ended-------------")

    def heuristicCost(self, wayPoint):
        return math.sqrt((endNode.x - wayPoint.x) ** 2 + (endNode.y - wayPoint.y) ** 2)


class WayPoint:

    def __init__(self, x, y):
        self.x = x
        self.y = y
        self.hCost = 0
        self.cost = 0
        self.gCost = 0
        self.fCost = 0

    # The waypoint's historical cost
    def calculateHCost(self, startNode):
        # The Manhattan Distance to the beginning
        self.hCost = (self.x - startNode.x) + (self.y - startNode.y)

    # Set the cost for a WayPoint
    def setCost(self, cost):
        # 100 for wall
        # 0 for empty
        # -1 for unknown
        self.cost = cost

    def calculateGCost(self, endNode):
        # Euclidian Distance to the end
        self.gCost = math.sqrt((endNode.x - self.x)**2 + (endNode.y - self.y)**2)

    def calculateFCost(self):
        self.fCost = self.gCost + self.hCost

    # Check if this node and compareNode are occupying the same space
    def isSame(self, compareNode):
        if( (compareNode.x == self.x) && (compareNode.y == self.y)):
            return True
        else:
            return False

if __name__ == '__main__':
    endNode = WayPoint(100,100)
    startNode = WayPoint(0,0)
    aStar = AStar(startNode,endNode)
    aStar.findPath()
