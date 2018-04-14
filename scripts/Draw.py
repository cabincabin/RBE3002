#!/usr/bin/env python

# File: Draw.py
# Author: Floris VanRossum
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

# Purpose: Draw a certain list of nodes on the gridmap on rviz
# topic: The topic to publish the grid to
# nodes: An array of nodes (waypoints) that need to be visualized
# distance: this distance between two nodes
def drawGrid(topic,nodes,distance):

        # Instantiate a new grid
        grid = GridCells()

        # put the nodes into the grid
        for i in range(0, len(nodes)):
            node = nodes[i]
            p = Point()
            p.x = node.point.x
            p.y = node.point.y
            p.z = -.01
            pose = PoseStamped()
            pose.pose.position = node.point
            grid.cells.append(p)

        # Set grid cell constants
        grid.cell_height = distance
        grid.cell_width = distance
        grid.header.frame_id = "map"

        # Create a new publisher and publish the grid
        tempPublisher = rospy.Publisher(topic, GridCells, None, queue_size=1)
        tempPublisher.publish(grid)