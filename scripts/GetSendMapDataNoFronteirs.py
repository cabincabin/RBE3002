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
import time
import numpy as np
from Draw import drawGrid
from std_msgs.msg import String

#THIS IS A NODE. TREAT AS SUCH
##########################################################

class GridSpacePathing:

    def __init__(self):
        #set up the position of therobot
        self._odom_list = tf.TransformListener()
        self._roll = 0
        self._pitch = 0
        self._yaw = 0
        #current robot orientation
        self._current = Pose()
        #robot's position
        self._robot = WayPoint(-1000000, -1000000)
        #the current goal as a waypoint
        self._goalWay = None
        #the current
        self._CurrGoal = None
        #update robot's position
        rospy.Timer(rospy.Duration(0.1), self.timerCallback)
        #size of the robot's width/length
        self._robotSize = .1
        #the current occupancy grid
        self._currmap = None
        #the current path as a set of poses
        self._currPath = None
        #A boolean, wait until the robot's position is set
        self.RobotPoseInit = False
        #only make one base mao
        self.UpdateOriginalMapOnce = True
        #list of all waypoints in the base map
        self._waypointlist = []
        #list of all occupancy grids through time
        self._allMaps = []
        #list of occupied grids
        self._OccGrids = GridCells()
        #is there a path that the robot is navigating to?
        self.IsPath = False
        self.front = GridCells()
        self.Unocc = GridCells()
        self.frontierList = []
        self.isCurrFrontGoal = False
        self.frontGoal = WayPoint(-10000,-10000)

        #subscribe to the map's occupancy grid.
        #set up the visualization for the grid and path
        rospy.Subscriber('/map', OccupancyGrid, self.getMapInfo, queue_size=1)
        self._showOccupied = rospy.Publisher('/nav_msgs/GridCellsOcc', GridCells, None, queue_size=1)
        self._ShowStart = rospy.Publisher('/nav_msgs/GridCellsStart', GridCells, None, queue_size=1)
        self._ShowEnd = rospy.Publisher('/nav_msgs/GridCellsEnd', GridCells, None, queue_size=1)
        self._ShowPathGrid = rospy.Publisher('/nav_msgs/GridCellsPath', GridCells, None, queue_size=1)
        self._ShowPathPath = rospy.Publisher('/Aplan', Path, None, queue_size=1)
        self._ShowFront = rospy.Publisher('/nav_msgs/GridCellsFrontier', GridCells, None, queue_size=1)
        self._Showunocc = self._ShowPathGrid = rospy.Publisher('/nav_msgs/GridCellsChecked', GridCells, None, queue_size=1)
        #print("here")
        rospy.Timer(rospy.Duration(1), self.CreateMapOccupancy) #will be useful for D*

        # Timers and Subscribers
        #when a nav goal is published, pathfind to this position.
        rospy.Subscriber('/move_base_simple/goal1', PoseStamped, self.PathToPos, queue_size=1)

    def PathToPos(self, goal):
        self._CurrGoal = goal
        #self.clearAllGrids()
        #get the goal position from the robot transformation
        self._odom_list.waitForTransform('map', '/base_footprint', rospy.Time(0), rospy.Duration(2.0))
        rospy.sleep(1.0)
        transGoal = self._odom_list.transformPose('map', goal)  # transform the nav goal from the global coordinate system to the robot's coordinate system
        closest = self._waypointlist[0]

        #create a goal waypoint
        self._goalWay = WayPoint(transGoal.pose.position.x, transGoal.pose.position.y)

        #finds nearest waypoint in the actual grid to neighbor to this point,
        #should probably be a new method

        #update the robot's position as it relates to waypoints
        self._robot = self.FindNNInWayPoints(WayPoint(self._current.position.x, self._current.position.y))
        for i in range(len(self._waypointlist)):
            if self._currmap.data[i] != 100 and self._waypointlist[i].calculateMDistance(self._goalWay) < closest.calculateMDistance(self._goalWay):
                closest = self._waypointlist[i]
                #print("")
        self._goalWay = closest



        #get the path
        star = AStar(self._robot, self._goalWay)
        path = star.findPath()

        # Drawing the path cells
        grid = GridCells()
        pathDisp = Path()
        reached = False
        self._currPath = []
        self._currPath.append(self._goalWay)
        #put the path into GridCells
        prevNode = self._goalWay
        #create the actual path of the robot from start to finish
        while not reached:
            node = path.get(prevNode)
            p = Point()
            p.x = node.point.x
            p.y = node.point.y
            p.z = 0
            #for each node in the path, if its next to an occupied location
            #push the point as far from the occupied location as possible.
            #this, as a side effect due to the 4 connected-ness will not be affected by walls on corners
            #createing a "smoothing" of the robot.
            if(len(node.connectedNodes) < 4):
                for n in node.connectedNodes:
                    if n.point.x > node.point.x -.0001 and n.point.x < node.point.x +.0001:
                        #add a displacement to the y offset in the direction of the previous node
                        #via unit vector
                        print("")
                        print(p.y)
                        p.y = p.y + (n.point.y - node.point.y)/2
                        print(p.y)

                    if n.point.y > node.point.y -.0001 and n.point.y < node.point.y +.0001:
                        # add a displacement to the y offset in the direction of the previous node
                        # via unit vector
                        p.x = p.x + (n.point.x - node.point.x)/2
            #add the nodes to the list of nodes.
            pose = PoseStamped()
            rospy.sleep(1.0)
            pose.header.frame_id = "map"
            pose.header.stamp = rospy.Time.now()
            pose.pose.position = p
            pathDisp.poses.append(pose)
            grid.cells.append(p)
            prevNode = node
            self._currPath.append(node)
            if node.isSame(self._robot):
                reached = True


        grid.cell_height = self._currmap.info.resolution*int(math.ceil(self._robotSize/self._currmap.info.resolution))
        grid.cell_width = self._currmap.info.resolution*int(math.ceil(self._robotSize/self._currmap.info.resolution))
        grid.header.frame_id = "map"
        pathDisp.header.frame_id = "map"


        pathDisp.poses.reverse()
        pathDisp.poses.append(goal)

        #calculate the angle of the robot
        for i in range(len(pathDisp.poses)-1):
                #calculate the angle and add it to the corrisponding node
                ang = math.atan2(-pathDisp.poses[i].pose.position.y+pathDisp.poses[i+1].pose.position.y, -pathDisp.poses[i].pose.position.x+pathDisp.poses[i+1].pose.position.x)
                quat = quaternion_from_euler(0,0,ang)
                pathDisp.poses[i].pose.orientation.x = quat[0]
                pathDisp.poses[i].pose.orientation.y = quat[1]
                pathDisp.poses[i].pose.orientation.z = quat[2]
                pathDisp.poses[i].pose.orientation.w = quat[3]

        #show the path
        pathDisp.poses.pop(0)
        self._ShowPathGrid.publish(grid)
        self._ShowPathPath.publish(pathDisp)

        self.drawStartAndEnd()
        self.IsPath = True

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

        gridStart.cell_height = self._currmap.info.resolution*int(math.ceil(self._robotSize/self._currmap.info.resolution))
        gridStart.cell_width = self._currmap.info.resolution*int(math.ceil(self._robotSize/self._currmap.info.resolution))
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

        gridEnd.cell_height = self._currmap.info.resolution*int(math.ceil(self._robotSize/self._currmap.info.resolution))
        gridEnd.cell_width = self._currmap.info.resolution*int(math.ceil(self._robotSize/self._currmap.info.resolution))
        gridEnd.header.frame_id = "map"

        self._ShowEnd.publish(gridEnd)

    def getMapInfo(self, currmap):
        #when the map changes, update the occupancy grid.
        print("here")
        print(currmap.data[0])
        #if currmap.header.seq == 0:
        #add to the list of all maps ever
        self._allMaps.append(currmap)
        self._currmap = currmap

        if len(self._allMaps)>3:
            #if there are more than the 3 original maps on generation
            #update the base maps occupancy to renav if needed
            self.UpdateMapOccupancy()

    def FindNNInWayPoints(self, waypoint):
        closest = self._waypointlist[0]
        # finds nearest waypoint in the actual grid to neighbor to this point,
        # could probably be done with Astar
        for i in range(len(self._waypointlist)):
            if self._currmap.data[i] < 70 and self._waypointlist[i].calculateMDistance(
                    waypoint) < closest.calculateMDistance(waypoint):
                closest = self._waypointlist[i]
        return closest

    def UpdateMapOccupancy(self):
        print("here5")
        #if the occupancy changes, then update the pathfinding
        CheckUpdatePath = False
        for wayp in self._waypointlist:
                if(wayp<70 and wayp != self._robot):
                    isNowOcc = False
                    isKnown = True
                    OccVal = 0
                    for ind in wayp.spaces:
                        if self._allMaps[-1].data[ind] >= 70:
                            isNowOcc=True

                        elif self._allMaps[-1].data[ind] < 0:
                            isKnown = False
                        if OccVal < self._allMaps[-1].data[ind]:
                            OccVal = self._allMaps[-1].data[ind]

                    if isNowOcc == True:

                        p = Point()
                        p.x = wayp.point.x
                        p.y = wayp.point.y
                        p.z = 0.1
                        #note that that this point is occupied for publishing
                        self._OccGrids.cells.append(p)

                        wayp._occ = OccVal
                        for neighbor in range(len(wayp.connectedNodes)):
                            wayp.connectedNodes[neighbor].connectedNodes.remove(wayp)
                        wayp.connectedNodes = []
                        CheckUpdatePath = True
                    elif isKnown:
                        wayp._occ = OccVal

        self._showOccupied.publish(self._OccGrids)
        print("here6")
        #if there is a path and the occupancy grid was updated
        #drawGrid('/nav_msgs/GridCellsStart',singleNodeArr,distance)

        if CheckUpdatePath and self.IsPath:
            #reset the pathfinding
            tempPublisher = rospy.Publisher('/AReset', Twist, None, queue_size=1)
            tempPublisher.publish(Twist())
            rospy.sleep(5)
            try:
                #rerun aStar and re drive
                for wayp in range(len(self._currPath)):
                    if self._currPath[wayp]._occ >= 70:
                        self.PathToPos(self._CurrGoal)
                        break
            except NameError:
                print("no Path Yet")


    def CreateMapOccupancy(self, evprent): #this should update all nodes and recompute path if needed
        if self._currmap != None and self.RobotPoseInit == True and self.UpdateOriginalMapOnce == True: #remove the only once boolean for D*, will wait until the robot's position is found
            rospy.Rate(1).sleep()

            grid = GridCells()

            #this is used for nearest neighbor method for finding the robot
            print(self._current.position.x, self._current.position.y)
            robot = WayPoint(self._current.position.x, self._current.position.y)

            NumOfOcc = int(math.ceil(self._robotSize/self._currmap.info.resolution))
            i = 0
            print(NumOfOcc)

            #for each item on the 2 dimentional array
            for r in range(int(math.floor(self._currmap.info.height/NumOfOcc))):
                for c in range(int(math.floor(self._currmap.info.width/NumOfOcc))):
                    IsOcc = False
                    spaces = []
                    #checks c space to find if the location is occupied and calculates where the waypoint is in 2d space
                    for j in range(r*NumOfOcc, (r*NumOfOcc+NumOfOcc+1), 1):
                        for k in range(c*NumOfOcc, (c*NumOfOcc+NumOfOcc+1), 1):
                            if j < self._currmap.info.height and k < self._currmap.info.width:
                                spaces.append(j*self._currmap.info.width + k)
                                if self._currmap.data[j*self._currmap.info.width + k] >= 70:
                                    IsOcc = True



                    #if occupied, add this to the gridcells to draw the occupancy grid
                    if (IsOcc == True):
                        #the position of the gridcell is the offset of the occupancy to the world + the size of the cells*num of cells to point, + half the size of a cell
                        #due to the way that the gridCells is drawn
                        p = Point()
                        p.x = self._currmap.info.origin.position.x + c*NumOfOcc*self._currmap.info.resolution + self._currmap.info.resolution*NumOfOcc/2
                        p.y = self._currmap.info.origin.position.y + r*NumOfOcc*self._currmap.info.resolution + self._currmap.info.resolution*NumOfOcc/2
                        p.z = 0
                        grid.cells.append(p)
                        grid.cell_height = self._currmap.info.resolution*NumOfOcc
                        grid.cell_width = self._currmap.info.resolution*NumOfOcc
                        grid.header.frame_id = "map"
                        currPoint = WayPoint(p.x, p.y, 100, spaces)
                        self._waypointlist.append(currPoint)

                    #for every empty space
                    else:
                        p = Point()
                        p.x = self._currmap.info.origin.position.x + c*NumOfOcc*self._currmap.info.resolution + self._currmap.info.resolution*NumOfOcc/2
                        p.y = self._currmap.info.origin.position.y + r*NumOfOcc*self._currmap.info.resolution + self._currmap.info.resolution*NumOfOcc/2
                        p.z = 0
                        # add the waypoint to the grid and see if it should be the robot's position via nearest neighbor
                        currPoint = WayPoint(p.x, p.y, -1, spaces)
                        self._waypointlist.append(currPoint)
                            #print("")
            #print(i)
            closest = self._waypointlist[0]
            # finds nearest waypoint in the actual grid to neighbor to this point,
            # could probably be done with Astar
            for i in range(len(self._waypointlist)):
                if self._waypointlist[i].calculateMDistance(robot) < closest.calculateMDistance(robot):
                    closest = self._waypointlist[i]
            #publish the grid
            #update info
            self._showOccupied.publish(grid)
            self._OccGrids = grid
            self.RobotPoseInit = False
            self.UpdateOriginalMapOnce = False
            self._robot = closest

            closest._occ = 0

            print("............")
            print(self._currmap.info.height*self._currmap.info.width)
            print(len(self._waypointlist*NumOfOcc*NumOfOcc))

            #use black magic to connect every point in the 2d grid to every adjacent point that is not occupied, from a
            #1 dimentional grid.
            for i in range(len(self._waypointlist)):
                #if the point is not occupied
                if(self._waypointlist[i]._occ < 70):
                    #connect it to the waypoint to the left of it, if that point is not occupied
                    if i % int(math.floor(self._currmap.info.width/NumOfOcc)) - 1 >= 0 and self._waypointlist[i-1]._occ < 70:
                        self._waypointlist[i].connectedNodes.append(self._waypointlist[i-1])
                        # connect it to the waypoint to the right of it, if that point is not occupied
                    if i % int(math.floor(self._currmap.info.width/NumOfOcc)) + 1 < int(math.floor(self._currmap.info.width/NumOfOcc)) and  self._waypointlist[i+1]._occ < 70:
                        self._waypointlist[i].connectedNodes.append(self._waypointlist[i+1])
                        # connect it to the waypoint below, if that point is not occupied
                    if i + int(math.floor(self._currmap.info.width/NumOfOcc)) < int(math.floor(self._currmap.info.height/NumOfOcc))*int(math.floor(self._currmap.info.width/NumOfOcc)) and self._waypointlist[i + int(math.floor(self._currmap.info.width/NumOfOcc))]._occ < 70:
                        self._waypointlist[i].connectedNodes.append(self._waypointlist[i+int(math.floor(self._currmap.info.width/NumOfOcc))])
                        # connect it to the waypoint above, if that point is not occupied
                    if i - int(math.floor(self._currmap.info.width/NumOfOcc)) >= 0 and self._waypointlist[i-int(math.floor(self._currmap.info.width/NumOfOcc))]._occ < 70:
                        self._waypointlist[i].connectedNodes.append(self._waypointlist[i-int(math.floor(self._currmap.info.width/NumOfOcc))])


    def timerCallback(self,evprent):
        # Called back rapidly to update odometry

        # Convert to global coordinates
        self._odom_list.waitForTransform('map', '/base_footprint', rospy.Time(0), rospy.Duration(1.0))
        (position, orientation) = self._odom_list.lookupTransform('map', '/base_footprint', rospy.Time(0))

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
