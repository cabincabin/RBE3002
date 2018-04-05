#!/usr/bin/env python

# File: lab2.py
# Author: Clayton Dembski
# Additional code from fvanrossum_lab2.py
# Class: RBE3002
# Project: Lab 2
# Professor: Professor Pinciroli


#####################IMPORTS##############################
import rospy, tf, copy, math, roslib, sys
from AStarTest import WayPoint
from geometry_msgs.msg import Point, Twist, Pose, PoseStamped
from nav_msgs.msg import OccupancyGrid, GridCells
from tf.transformations import euler_from_quaternion
import numpy as np
from std_msgs.msg import String
##########################################################

class GridSpacePathing:

    def __init__(self):
        self._odom_list = tf.TransformListener()
        self._roll = 0
        self._pitch = 0
        self._yaw = 0
        self._current = Pose()
        rospy.Timer(rospy.Duration(0.1), self.timerCallback)
        self._currmap = None
        self.boolthing = False
        self.boolthing2 = True
        self._waypointlist = []
        rospy.Subscriber('/map', OccupancyGrid, self.getMapInfo, queue_size=10)
        self._pub = rospy.Publisher('/nav_msgs/GridCells', GridCells, None, queue_size=1)
        print("here")
        rospy.Timer(rospy.Duration(1), self.UpdateMapOccupancy) #will be useful for D*

        # Timers and Subscribers

        rospy.Subscriber('/move_base_simple/goal', PoseStamped, self.PathToPos, queue_size=1)


    def getMapInfo(self, currmap):
        print("here")
        print(currmap.data[0])
        self._currmap = currmap



    def UpdateMapOccupancy(self, evprent): #this should update all nodes and recompute path if needed
        if self._currmap != None and self.boolthing == True and self.boolthing2 == True: #remove the only once boolean for D*
            rospy.Rate(1).sleep()
            grid = GridCells()

            closest = WayPoint(-10000,-10000)
            print(self._current.position.x, self._current.position.y)
            robot = WayPoint(self._current.position.x, self._current.position.y)
            for r in range(self._currmap.info.height):
                for c in range(self._currmap.info.width):
                    if self._currmap.data[self._currmap.info.width*r+c] == 100:
                        p = Point()
                        p.x = self._currmap.info.origin.position.x + c*self._currmap.info.resolution + self._currmap.info.resolution/2
                        p.y = self._currmap.info.origin.position.y + r*self._currmap.info.resolution + self._currmap.info.resolution/2
                        p.z = 0
                        grid.cells.append(p)
                        grid.cell_height = self._currmap.info.resolution
                        grid.cell_width = self._currmap.info.resolution
                        grid.header.frame_id = "map"
                        currPoint = WayPoint(p.x, p.y)
                        self._waypointlist.append(currPoint)
                    else:
                        p = Point()
                        p.x = self._currmap.info.origin.position.x + c*self._currmap.info.resolution + self._currmap.info.resolution/2
                        p.y = self._currmap.info.origin.position.y + r*self._currmap.info.resolution + self._currmap.info.resolution/2
                        p.z = 0
                        currPoint = WayPoint(p.x, p.y)
                        self._waypointlist.append(currPoint)
                        if currPoint.calculateMDistance(robot) < closest.calculateMDistance(robot):
                            closest = currPoint
                            print("")

            self._pub.publish(grid)
            self.boolthing = False
            self.boolthing2 = False

            print("............")
            print(self._currmap.info.height*self._currmap.info.width)
            print(len(self._waypointlist))

            for i in range(self._currmap.info.height*self._currmap.info.width):
                if(self._currmap.data[i] != 100):
                    if i % self._currmap.info.width -1 >= 0 and self._currmap.data[i-1] != 100:
                        self._waypointlist[i].connectedNodes.append(self._waypointlist[i-1])
                    if i % self._currmap.info.width + 1 < self._currmap.info.width and self._currmap.data[i+1] != 100:
                        self._waypointlist[i].connectedNodes.append(self._waypointlist[i+1])
                    if i + self._currmap.info.width < self._currmap.info.height*self._currmap.info.width and self._currmap.data[i+self._currmap.info.width] != 100:
                        self._waypointlist[i].connectedNodes.append(self._waypointlist[i+self._currmap.info.width])
                    if i - self._currmap.info.width >= 0 and self._currmap.data[i-self._currmap.info.width] != 100:
                        self._waypointlist[i].connectedNodes.append(self._waypointlist[i-self._currmap.info.width])


            for i in range(self._currmap.info.height*self._currmap.info.width):
                if self._waypointlist[i] == closest:
                    print "-",
                elif (i+1) % self._currmap.info.width !=0:
                    if len(self._waypointlist[i].connectedNodes) == 0:
                        print " ",
                    else:
                        print len(self._waypointlist[i].connectedNodes),
                else:
                    if len(self._waypointlist[i].connectedNodes) == 0:
                        print " "
                    else:
                        print len(self._waypointlist[i].connectedNodes)


    def timerCallback(self,evprent):
        # Called back rapidly to update odometry

        # Convert to global coordinates
        self._odom_list.waitForTransform('/odom', '/base_footprint', rospy.Time(0), rospy.Duration(1.0))
        (position, orientation) = self._odom_list.lookupTransform('/odom', '/base_footprint', rospy.Time(0))

        # Update the data
    	self._current.position.x = position[0]
        self._current.position.y = position[1]
        self._current.orientation.x = orientation[0]
        self._current.orientation.y = orientation[1]
        self._current.orientation.z = orientation[2]
        self._current.orientation.w = orientation[3]

        # Update the quaternion
        q = [self._current.orientation.x,
                self._current.orientation.y,
                self._current.orientation.z,
                self._current.orientation.w]

        (roll, pitch, yaw) = euler_from_quaternion(q)

        self._roll = roll
        self._pitch = pitch

        # Convert the yaw to a logical angle
        if(yaw < 0 ):
            yaw = yaw + (math.pi*2)

        # Update yaw var
        self._yaw = yaw
        self.boolthing = True





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
