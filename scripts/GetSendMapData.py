#!/usr/bin/env python

# File: lab2.py
# Author: Clayton Dembski
# Class: RBE3002
# Project: Lab 2
# Professor: Professor Pinciroli


#####################IMPORTS##############################
import rospy, tf, copy, math, roslib
from geometry_msgs.msg import Point, Twist, Pose, PoseStamped
from nav_msgs.msg import OccupancyGrid, GridCells
from tf.transformations import euler_from_quaternion
import numpy as np
from std_msgs.msg import String
##########################################################

class GridSpacePathing:

    def __init__(self):
        self._currmap = None
        rospy.Subscriber('/map', OccupancyGrid, self.getMapInfo, queue_size=10)
        self._pub = rospy.Publisher('/nav_msgs/GridCells', GridCells, None, queue_size=1)
        rospy.Timer(rospy.Duration(5), self.timerCallback)



    def getMapInfo(self, currmap):
        print("here")
        print(currmap.data[0])
        self._currmap = currmap


    def timerCallback(self, evprent):
        if self._currmap != None:
            rospy.Rate(10).sleep()
            grid = GridCells()

            for r in range(self._currmap.info.height):
                for c in range(self._currmap.info.width):
                    if self._currmap.data[self._currmap.info.width*r+c] == 100:
                        p = Point()
                        p.x = self._currmap.info.origin.position.x + c*self._currmap.info.resolution + self._currmap.info.resolution/2
                        p.y = self._currmap.info.origin.position.y + r*self._currmap.info.resolution + self._currmap.info.resolution/2
                        p.z = 0
                        grid.cells.append(p)
                        grid.cell_height = self._currmap.info.resolution
                        grid.cell_width = .25
                        grid.header.frame_id = "map"
            self._pub.publish(grid)




if __name__ == '__main__':

    rospy.init_node('AStarPathing')


#    try:
#        # transform the nav goal from the global coordinate system to the robot's coordinate system
#        transGoal = self._odom_list.transformPose('/odometry', goal)
#        (trans,rot) = listener.lookupTransform(turtle, rospy.Time(0))
#    except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
#        pass

    #test function calls here
    rospy.Rate(1).sleep()
    GridSpacePathing()
    while not rospy.is_shutdown():

        pass

    # Turn off motors
