#!/usr/bin/env python

# File: lab2.py
# Author: Clayton Dembski
# Additional code from fvanrossum_lab2.py
# Class: RBE3002
# Project: Lab 2
# Professor: Professor Pinciroli


#####################IMPORTS##############################
import rospy, tf, copy, math, roslib, sys
from AStarTest import WayPoint, AStar
from geometry_msgs.msg import Point, Twist, Pose, PoseStamped
from nav_msgs.msg import OccupancyGrid, GridCells, Path
from tf.transformations import euler_from_quaternion, quaternion_from_euler
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
        self._robot = WayPoint(-10000, -10000)
        self.goalWay = WayPoint(-10000, -10000)
        rospy.Timer(rospy.Duration(0.1), self.timerCallback)
        self._currmap = None
        self.boolthing = False
        self.boolthing2 = True
        self._waypointlist = []
        rospy.Subscriber('/map', OccupancyGrid, self.getMapInfo, queue_size=10)
        self._pub = rospy.Publisher('/nav_msgs/GridCells', GridCells, None, queue_size=1)
        self._pub1 = rospy.Publisher('/nav_msgs/GridCells1', GridCells, None, queue_size=1)
        self._pub2 = rospy.Publisher('/nav_msgs/GridCells2', GridCells, None, queue_size=1)
        self._pub4 = rospy.Publisher('/nav_msgs/GridCells3', GridCells, None, queue_size=1)
        self._pub3 = rospy.Publisher('/Aplan', Path, None, queue_size=1)
        print("here")
        rospy.Timer(rospy.Duration(1), self.UpdateMapOccupancy) #will be useful for D*

        # Timers and Subscribers

        rospy.Subscriber('/move_base_simple/goal', PoseStamped, self.PathToPos, queue_size=1)

    def PathToPos(self, goal):

        self._odom_list.waitForTransform('/odom', '/base_footprint', rospy.Time(0), rospy.Duration(2.0))
        rospy.sleep(1.0)
        transGoal = self._odom_list.transformPose('/odom',goal)  # transform the nav goal from the global coordinate system to the robot's coordinate system
        closest = WayPoint(-10000, -10000)#ERROR LATER
        self.goalWay = WayPoint(transGoal.pose.position.x,transGoal.pose.position.y)
        for i in range(self._currmap.info.height * self._currmap.info.width):
            if self._currmap.data[i] != 100 and self._waypointlist[i].calculateMDistance(self.goalWay) < closest.calculateMDistance(self.goalWay):
                closest = self._waypointlist[i]
                print("")
        self.goalWay = closest

        for i in range(self._currmap.info.height * self._currmap.info.width):
            if self._waypointlist[i] == self._robot:
                print "_",
            elif self._waypointlist[i] == self.goalWay:
                print "'",
            elif (i + 1) % self._currmap.info.width != 0:
                if len(self._waypointlist[i].connectedNodes) == 0:
                    print " ",
                else:
                    print len(self._waypointlist[i].connectedNodes),
            else:
                if len(self._waypointlist[i].connectedNodes) == 0:
                    print " "
                else:
                    print len(self._waypointlist[i].connectedNodes)

        star = AStar(self._robot, self.goalWay)
        path = star.findPath()

        # Drawing the path cells
        grid = GridCells()
        pathDisp = Path()
        reached = False

        prevNode = self.goalWay
        while not reached:
            node = path.get(prevNode)
            p = Point()
            p.x = node.point.x
            p.y = node.point.y
            p.z = 0
            pose = PoseStamped()
            pose.pose.position = node.point
            pathDisp.poses.append(pose)
            grid.cells.append(p)
            prevNode = node

            if node.isSame(self._robot):
                reached = True

        grid.cell_height = self._currmap.info.resolution
        grid.cell_width = self._currmap.info.resolution
        grid.header.frame_id = "map"
        pathDisp.header.frame_id = "map"

        #self._pub3.publish(pathDisp)
        pathDisp2 = Path()
        pathDisp2.poses.append(goal)
        prevang = -1000000
        for i in range(len(pathDisp.poses)-1):
            if i > 0:
                ang = math.atan2(pathDisp.poses[i].pose.position.y-pathDisp.poses[i-1].pose.position.y, pathDisp.poses[i].pose.position.x-pathDisp.poses[i-1].pose.position.x)
                quat = quaternion_from_euler(0,0,ang+math.pi)
                pathDisp.poses[i+1].pose.orientation.x = quat[0]
                pathDisp.poses[i+1].pose.orientation.y = quat[1]
                pathDisp.poses[i+1].pose.orientation.z = quat[2]
                pathDisp.poses[i+1].pose.orientation.w = quat[3]
                # if i > 1:
                #     pathDisp.poses[i-2].pose.position=pathDisp.poses[i-1].pose.position
                if not ((ang > prevang-.25) and (ang < prevang+.25)):
                    p = PoseStamped()
                    p.pose.position = pathDisp.poses[i-1].pose.position
                    p.pose.orientation = pathDisp.poses[i].pose.orientation
                    pathDisp2.poses.append(p)
                    prevang = ang
        pathDisp2.header.frame_id = "map"

        self._pub4.publish(grid)
        self._pub3.publish(pathDisp2)



        self.drawStartEnd()

    def drawStartEnd(self):
        # Drawing the start cell
        gridStart = GridCells()

        node = self._robot
        p = Point()
        p.x = node.point.x
        p.y = node.point.y
        p.z = 1
        gridStart.cells.append(p)
        prevNode = node

        gridStart.cell_height = self._currmap.info.resolution
        gridStart.cell_width = self._currmap.info.resolution
        gridStart.header.frame_id = "map"


        self._pub1.publish(gridStart)

        gridEnd = GridCells()

        node = self.goalWay
        p = Point()
        p.x = node.point.x
        p.y = node.point.y
        p.z = 0
        gridEnd.cells.append(p)

        gridEnd.cell_height = self._currmap.info.resolution
        gridEnd.cell_width = self._currmap.info.resolution
        gridEnd.header.frame_id = "map"

        self._pub2.publish(gridEnd)

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
            grid.cells.append(closest.point)
            self._pub.publish(grid)
            self.boolthing = False
            self.boolthing2 = False
            self._robot = closest

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
