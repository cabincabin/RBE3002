#!/usr/bin/env python

# File: lab2.py
# Author: Clayton Dembski, Floris VanRossum
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
from Draw import clearMap,drawGrid
from std_msgs.msg import String
##########################################################

class GridSpacePathing:

    def __init__(self):
        #set up the position of therobot
        self._odom_list = tf.TransformListener()
        self._roll = 0
        self._pitch = 0
        self._yaw = 0
        self._current = Pose()
        self._robot = WayPoint(-1000000, -1000000)
        self._goalWay = WayPoint(-1000000, -1000000)
        rospy.Timer(rospy.Duration(0.1), self.timerCallback)

        self._currmap = None
        self.RobotPoseInit = False
        self.UpdatePathOnce = True
        self._waypointlist = []

        #subscribe to the map's occupancy grid.
        #set up the visualization for the grid and path
        rospy.Subscriber('/map', OccupancyGrid, self.getMapInfo, queue_size=10)
        self._showOccupied = rospy.Publisher('/nav_msgs/GridCellsOcc', GridCells, None, queue_size=1)
        self._ShowStart = rospy.Publisher('/nav_msgs/GridCellsStart', GridCells, None, queue_size=1)
        self._ShowEnd = rospy.Publisher('/nav_msgs/GridCellsEnd', GridCells, None, queue_size=1)
        self._ShowPathGrid = rospy.Publisher('/nav_msgs/GridCellsPath', GridCells, None, queue_size=1)
        self._ShowPathPath = rospy.Publisher('/Aplan', Path, None, queue_size=1)
        print("here")
        rospy.Timer(rospy.Duration(1), self.UpdateMapOccupancy) #will be useful for D*

        # Timers and Subscribers
        #when a nav goal is published, pathfind to this position.
        rospy.Subscriber('/move_base_simple/goal', PoseStamped, self.PathToPos, queue_size=1)

    def PathToPos(self, goal):
        #self.clearAllGrids()
        #get the goal position from the robot transformation
        self._odom_list.waitForTransform('/odom', '/base_footprint', rospy.Time(0), rospy.Duration(2.0))
        rospy.sleep(1.0)
        transGoal = self._odom_list.transformPose('map', goal)  # transform the nav goal from the global coordinate system to the robot's coordinate system
        closest = self._waypointlist[0]

        #create a goal waypoint
        self._goalWay = WayPoint(transGoal.pose.position.x, transGoal.pose.position.y)

        #finds nearest waypoint in the actual grid toneighbor to this point,
        #should probably be a new method
        for i in range(self._currmap.info.height * self._currmap.info.width):
            if self._currmap.data[i] != 100 and self._waypointlist[i].calculateMDistance(self._goalWay) < closest.calculateMDistance(self._goalWay):
                closest = self._waypointlist[i]
                print("")
        self._goalWay = closest

        # #for every waypoint in the grid
        # #print out the waypoint to the user
        # for i in range(self._currmap.info.height * self._currmap.info.width):
        #     #print the robot's position
        #     if self._waypointlist[i] == self._robot:
        #         print "_",
        #     #print the goal's position
        #     elif self._waypointlist[i] == self._goalWay:
        #         print "'",
        #     #print the number of nodes each node is connected to
        #     elif (i + 1) % self._currmap.info.width != 0:
        #         if len(self._waypointlist[i].connectedNodes) == 0:
        #             print " ",
        #         else:
        #             print len(self._waypointlist[i].connectedNodes),
        #     else:
        #         if len(self._waypointlist[i].connectedNodes) == 0:
        #             print " "
        #         else:
        #             print len(self._waypointlist[i].connectedNodes)

        #get the path
        star = AStar(self._robot, self._goalWay, self._currmap.info.resolution)
        path = star.findPath()

        # Drawing the path cells
        grid = GridCells()
        pathDisp = Path()
        reached = False

        #put the path into GridCells
        prevNode = self._goalWay
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
        #get the
        # pathDisp2 = Path()
        # pathDisp2.poses.append(goal)
        # prevang = -1000000

        pathDisp.poses.reverse()
        pathDisp.poses.append(goal)



        #get only the intermediate changes
        for i in range(len(pathDisp.poses)-1):
            if i > 0:
                #calculate the angle and add it to the corrisponding node
                ang = math.atan2(-pathDisp.poses[i].pose.position.y+pathDisp.poses[i+1].pose.position.y, -pathDisp.poses[i].pose.position.x+pathDisp.poses[i+1].pose.position.x)
                quat = quaternion_from_euler(0,0,ang)
                pathDisp.poses[i].pose.orientation.x = quat[0]
                pathDisp.poses[i].pose.orientation.y = quat[1]
                pathDisp.poses[i].pose.orientation.z = quat[2]
                pathDisp.poses[i].pose.orientation.w = quat[3]
                # if i > 1:
                #     pathDisp.poses[i-2].pose.position=pathDisp.poses[i-1].pose.position
                # only add intermediate poses: ones where the angle changes
                # if not ((ang > prevang-.25) and (ang < prevang+.25)):
                #     p = PoseStamped()
                #     p.pose.position = pathDisp.poses[i-1].pose.position
                #     p.pose.orientation = pathDisp.poses[i].pose.orientation
                #     pathDisp2.poses.append(p)
                #     prevang = ang
        #pathDisp2.header.frame_id = "map"

        #show the path
        self._ShowPathGrid.publish(grid)
        self._ShowPathPath.publish(pathDisp)

        self.drawStartAndEnd()

    def drawStartAndEnd(self):
        # Drawing the start cell
        gridStart = GridCells()

        node = self._robot
        p = Point()
        p.x = node.point.x
        p.y = node.point.y
        p.z = 0
        gridStart.cells.append(p)
        prevNode = node

        gridStart.cell_height = self._currmap.info.resolution
        gridStart.cell_width = self._currmap.info.resolution
        gridStart.header.frame_id = "map"
        self._ShowStart.publish(gridStart)

        #Draw the ending cell
        gridEnd = GridCells()

        node = self._goalWay
        p = Point()
        p.x = node.point.x
        p.y = node.point.y
        p.z = 0
        gridEnd.cells.append(p)

        gridEnd.cell_height = self._currmap.info.resolution
        gridEnd.cell_width = self._currmap.info.resolution
        gridEnd.header.frame_id = "map"

        self._ShowEnd.publish(gridEnd)

    def getMapInfo(self, currmap):
        #when the map changes, update the occupancy grid.
        print("here")
        print(currmap.data[0])
        self._currmap = currmap

    # def clearAllGrids(self):
    #     clearMap('/nav_msgs/GridCellsEnd')
    #     clearMap('/nav_msgs/GridCellsStart')
    #     clearMap('/nav_msgs/GridCellsPath')
    #     emptyPath = Path()
    #     tempPublisher = rospy.Publisher('APlan', Path, queue_size = 1)
    #     tempPublisher.publish(emptyPath)

    def UpdateMapOccupancy(self, evprent): #this should update all nodes and recompute path if needed
        if self._currmap != None and self.RobotPoseInit == True and self.UpdatePathOnce == True: #remove the only once boolean for D*, will wait until the robot's position is found
            rospy.Rate(1).sleep()
            grid = GridCells()

            #this is used for nearest neighbor method for finding the robot
            closest = WayPoint(-10000,-10000)
            print(self._current.position.x, self._current.position.y)
            robot = WayPoint(self._current.position.x, self._current.position.y)

            #for each item on the 2 dimentional array
            for r in range(self._currmap.info.height):
                for c in range(self._currmap.info.width):
                    #if occupied, add this to the gridcells to draw the occupancy grid
                    if self._currmap.data[self._currmap.info.width*r+c] == 100:
                        p = Point()
                        #the position of the gridcell is the offset of the occupancy to the world + the size of the cells*num of cells to point, + half the size of a cell
                        #due to the way that the gridCells is drawn
                        p.x = self._currmap.info.origin.position.x + c*self._currmap.info.resolution + self._currmap.info.resolution/2
                        p.y = self._currmap.info.origin.position.y + r*self._currmap.info.resolution + self._currmap.info.resolution/2
                        p.z = 0
                        grid.cells.append(p)
                        grid.cell_height = self._currmap.info.resolution
                        grid.cell_width = self._currmap.info.resolution
                        grid.header.frame_id = "map"
                        currPoint = WayPoint(p.x, p.y)
                        self._waypointlist.append(currPoint)
                    #for every empty space
                    else:
                        p = Point()
                        p.x = self._currmap.info.origin.position.x + c*self._currmap.info.resolution + self._currmap.info.resolution/2
                        p.y = self._currmap.info.origin.position.y + r*self._currmap.info.resolution + self._currmap.info.resolution/2
                        p.z = 0
                        # add the waypoint to the grid and see if it should be the robot's position via nearest neighbor
                        currPoint = WayPoint(p.x, p.y)
                        self._waypointlist.append(currPoint)
                        if currPoint.calculateMDistance(robot) < closest.calculateMDistance(robot):
                            closest = currPoint
                            #print("")

            #publish the grid
            #update info
            grid.cells.append(closest.point)
            self._showOccupied.publish(grid)
            self.RobotPoseInit = False
            self.UpdatePathOnce = False
            self._robot = closest

            print("............")
            print(self._currmap.info.height*self._currmap.info.width)
            print(len(self._waypointlist))

            #use black magic to connect every point in the 2d grid to every adjacent point that is not occupied, from a
            #1 dimentional grid.
            for i in range(self._currmap.info.height*self._currmap.info.width):
                #if the point is not occupied
                if(self._currmap.data[i] != 100):
                    #connect it to the waypoint to the left of it, if that point is not occupied
                    if i % self._currmap.info.width -1 >= 0 and self._currmap.data[i-1] != 100:
                        self._waypointlist[i].connectedNodes.append(self._waypointlist[i-1])
                        # connect it to the waypoint to the right of it, if that point is not occupied
                    if i % self._currmap.info.width + 1 < self._currmap.info.width and self._currmap.data[i+1] != 100:
                        self._waypointlist[i].connectedNodes.append(self._waypointlist[i+1])
                        # connect it to the waypoint below, if that point is not occupied
                    if i + self._currmap.info.width < self._currmap.info.height*self._currmap.info.width and self._currmap.data[i+self._currmap.info.width] != 100:
                        self._waypointlist[i].connectedNodes.append(self._waypointlist[i+self._currmap.info.width])
                        # connect it to the waypoint above, if that point is not occupied
                    if i - self._currmap.info.width >= 0 and self._currmap.data[i-self._currmap.info.width] != 100:
                        self._waypointlist[i].connectedNodes.append(self._waypointlist[i-self._currmap.info.width])

            # #for every item in the grid print out the grid to the console
            # for i in range(self._currmap.info.height*self._currmap.info.width):
            #     #draw the robot as
            #     if self._waypointlist[i] == closest:
            #         print "_",
            #     elif (i+1) % self._currmap.info.width !=0:
            #         if len(self._waypointlist[i].connectedNodes) == 0:
            #             print " ",
            #         else:
            #             print len(self._waypointlist[i].connectedNodes),
            #     else:
            #         if len(self._waypointlist[i].connectedNodes) == 0:
            #             print " "
            #         else:
            #             print len(self._waypointlist[i].connectedNodes)


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
        self.RobotPoseInit = True





if __name__ == '__main__':

    rospy.init_node('AStarPathing')
    #test function calls here
    rospy.Rate(1).sleep()
    #init the content of the node
    GridSpacePathing()
    while not rospy.is_shutdown():

        pass

    # Turn off motors
