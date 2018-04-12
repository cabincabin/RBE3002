#!/usr/bin/env python

# File: Draw.py
# Author: Clayton Dembski, Floris VanRossum
# Class: RBE3002
# Project: Lab 4
# Professor: Professor Pinciroli


#####################IMPORTS##############################
import rospy, tf, copy, math, roslib, sys
from geometry_msgs.msg import Point, Twist, Pose, PoseStamped
from nav_msgs.msg import OccupancyGrid, GridCells, Path
from tf.transformations import euler_from_quaternion, quaternion_from_euler
import numpy as np
from std_msgs.msg import String
##########################################################

def drawGrid(topic,nodes,distance):
    #clearMap(topic)
    if len(nodes) > 2:
        grid = GridCells()
        pathDisp = Path()
        reached = False

        # put the path into GridCells

        distance_X = abs(nodes[0].point.x - nodes[1].point.x)
        distance_Y = abs(nodes[0].point.y - nodes[1].point.y)

        dist = math.sqrt((distance_X ** 2) + (distance_Y ** 2))

        for i in range(0, len(nodes)):
            node = nodes[i]
            p = Point()
            p.x = node.point.x
            p.y = node.point.y
            p.z = -.01
            pose = PoseStamped()
            pose.pose.position = node.point
            pathDisp.poses.append(pose)
            grid.cells.append(p)

        grid.cell_height = distance
        grid.cell_width = distance

        grid.header.frame_id = "map"

        tempPublisher = rospy.Publisher(topic, GridCells, None, queue_size=1)
        tempPublisher.publish(grid)

def clearMap(topic):
    emptyGrid = GridCells()
    tempPublisher = rospy.Publisher(topic, GridCells, None, queue_size=1)
    tempPublisher.publish(emptyGrid)
