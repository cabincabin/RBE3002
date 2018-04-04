# !/usr/bin/env python
# Clayton Dembski, Floris van Rossum
# RBE3002 - Team 3
# Lab 3 Code

import rospy, tf, copy, math

from geometry_msgs.msg import Twist, Pose, PoseStamped
from tf.transformations import euler_from_quaternion
import math

import numpy as np
from std_msgs.msg import String


# h(n) = Manhattan distance to start
# g(n) = Euclidian distance to goal

class AStar:
    def __init__(self, graph, startNode, endNode):
        # Put initialization here
        self.graph = graph # Contains the map of all waypoints
        self.startNode = startNode # The starting waypoint
        self.endNode = endNode # The ending waypoint

    # Find the best path
    def findAStar():
        evaluated = []
        notEvaluated = [start]
        previousSteps = []

        gCostGraph = map with default value 100 # TODO What is this?

        gCostGraph[startNode] = 0 # Cost of travelling from start node to start node is 0

        fCostGraph[startNode] = heuristicCost(startNode) # Calculate the heuristic cost

        # evaluating = []

        while len(notEvaluated) != 0:
            

    def heuristicCost(wayPoint):
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
        self.hCost = (x - startNode.x) + (y - startNode.y)

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
