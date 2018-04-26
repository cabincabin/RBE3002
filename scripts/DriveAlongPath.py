#!/usr/bin/env python

# File: lab2.py
# Author: Floris van Rossum
# Author: Clayton Dembski
# Class: RBE3002
# Professor: Professor Pinciroli


#####################IMPORTS##############################
import rospy, tf, copy, math, roslib

from geometry_msgs.msg import Twist, Pose, PoseStamped
from nav_msgs.msg import Path
from tf.transformations import euler_from_quaternion
import numpy as np
from std_msgs.msg import String
##########################################################

class Robot:

    def __init__(self):
        # Class initialization

        # Variables
        # Toggle between simulation and turtlebot3
        self._pub = rospy.Publisher('cmd_vel', Twist, None, queue_size = 1)
        # self._pub = rospy.Publisher('/cmd_vel', Twist, None, queue_size = 1)
        self._odom_list = tf.TransformListener()
        self._roll = 0
        self._pitch = 0
        self._yaw = 0
        self._current = Pose()

        self.interrupt = False

        # Timers and Subscribers
        rospy.Timer(rospy.Duration(.025), self.timerCallback)
        rospy.Subscriber('/Aplan', Path, self.navToPose, queue_size = 1)
        #rospy.Subscriber('/AReset', Twist, self.toggleInterrupt, queue_size=1)

    def toggleInterrupt(self, evprent):
        self.interrupt = True
        print("interrupted")

    def navToPose(self, goal):
        # Go to a specific location and assume orientation
        self.interrupt = False
        # Receives a goal: Pose()
        # position: x, y, z
        # orientation: x, y, z, w
        print("here1")
        # Convert goal to robot coordinates
        for pathIndex in range(len(goal.poses)):
            # if pathIndex == len(goal.poses)-1:
            #     distthresh = .1
            # else:
            distthresh = .193

            self._odom_list.waitForTransform('/odom', '/base_footprint', rospy.Time(0), rospy.Duration(2.0))
            rospy.sleep(1.0)
            goal.poses[pathIndex].header.stamp = rospy.Time.now()
            rospy.sleep(1.0)
            transGoal = self._odom_list.transformPose('/odom', goal.poses[pathIndex]) # transform the nav goal from the global coordinate system to the robot's coordinate system

            # Destination data
            x_dest = transGoal.pose.position.x
            y_dest = transGoal.pose.position.y
            angle_dest = transGoal.pose.orientation.z

            # Current data
            x_current = self._current.position.x
            y_current = self._current.position.y
            yaw_current = self._yaw

            # Calculate the angle to turn to in order to drive to point
            temp_angle = math.atan2((y_dest - y_current),(x_dest - x_current))

            if(temp_angle < 0):
                temp_angle = temp_angle + (2*math.pi)
            angle_to_dest = temp_angle - yaw_current

            print('Angle to Travel: ' + str(angle_to_dest))

            # Calculate the distance to drive
            distance_to_dest = math.sqrt((x_dest - x_current) ** 2 + (y_dest - y_current) ** 2)

            yaw_current = self._yaw

            # Rotate to face the point
            self.rotate(angle_to_dest)

            # Keep driving to the point in small increments, adjusts if heading
            # is not correct, loops until threshold is reached
            # dirX = (transGoal.pose.position.x - self._current.position.x) / abs(
            #     (transGoal.pose.position.x - self._current.position.x))
            # dirY = (transGoal.pose.position.Y - self._current.position.Y) / abs(
            #     (transGoal.pose.position.Y - self._current.position.Y))

            while(distance_to_dest > distthresh) and not self.interrupt:
                x_dest = transGoal.pose.position.x
                y_dest = transGoal.pose.position.y

                x_current = self._current.position.x
                y_current = self._current.position.y
                yaw_current = self._yaw

                angle_dest = transGoal.pose.orientation.z

                temp_angle = math.atan2((y_dest - y_current),(x_dest - x_current))

                if(temp_angle < 0):
                    temp_angle = temp_angle + (2*math.pi)
                angle_to_dest = temp_angle - yaw_current

                distance_to_dest = math.sqrt((x_dest - x_current) ** 2 + (y_dest - y_current) ** 2)

                print('rotating to destination')

                self.rotate(angle_to_dest)

                print('going to destination')
                self.driveStraight(0.075, distance_to_dest/2.0)



                yaw_current = self._yaw

                rospy.sleep(0.1)
            self.spinWheels(0.05, 0.05, 0.01)
            # Final rotation to face the final orientation
            if pathIndex == len(goal.poses) -1:

                q = [transGoal.pose.orientation.x,
                         transGoal.pose.orientation.y,
                         transGoal.pose.orientation.z,
                         transGoal.pose.orientation.w]

                # convert the quaternion to roll pitch yaw
                (roll, pitch, yawFinal) = euler_from_quaternion(q)

                angle_dest = yawFinal

                final_to_angle = angle_dest - yaw_current

                self.rotate(final_to_angle)

        # Arrived at the point
        print('Arrived')
        self.interrupt = False

    # Drive straight for a certain distance
    def driveStraight(self, speed, distance):
        # Set the beginning values
        beginning_x = self._current.position.x
        beginning_y = self._current.position.y
        beginning_yaw = self._yaw
        print('X Curr: ' + str(beginning_x))
        print('Y Curr: ' + str(beginning_y))

        # Calculate the location of destination
        dest_x = beginning_x + (distance * math.cos(beginning_yaw))
        dest_y = beginning_y + (distance * math.sin(beginning_yaw))
        print('X Dest: ' + str(dest_x))
        print('Y Dest: ' + str(dest_y))

        # Calculate the distance to travel to destination
        distanceTo = math.sqrt( ( -self._current.position.x + dest_x) ** 2
            + ( -self._current.position.y + dest_y) ** 2)

        # Incrementally approach the point until the destination has been reached
        thresh = 0.1
        while distanceTo > thresh and not self.interrupt:
            distanceTo = math.sqrt( (-self._current.position.x + dest_x) ** 2
                + (-self._current.position.y + dest_y) ** 2)
            # Print out the distance left to travel for debugging
            print('distanceTo: ' + str(distanceTo))
            self.spinWheels(speed,speed,0.01)
            rospy.sleep(0.01)


    # Move some wheels
    def spinWheels(self, v_left, v_right, time):
        diameter = 0.23 # based on wheel track from https://yujinrobot.github.io/kobuki/doxygen/enAppendixKobukiParameters.html

        omega = (v_right - v_left) / 2

        if (v_left == v_right):
            v = v_left
        else:
            r = (diameter / 2) * ((v_right + v_left) / (v_right - v_left))
            v = r * omega # Calculate the total velocity

        driveStartTime = rospy.Time.now().secs

        # Creating and publishing the twist message
        twist_message = Twist()
        twist_message.linear.x = v
        twist_message.angular.z = omega
        self._pub.publish(twist_message)

        # Calculate the current time and find out the passed time
        driveStartTime = rospy.get_time()
        rospy.sleep(0.01)
        updatedTime = rospy.get_time()
        rospy.sleep(0.01)
        driveStartTime = rospy.get_time()
        rospy.sleep(0.01)
        passedTime = updatedTime - driveStartTime

        # Debug print statements
        # print('Initial Difference: ' + str(passedTime))
        # print('Start: ' + str(driveStartTime))

        # Continually publish messages until destination is reached
        while passedTime < (time) and not self.interrupt:
            # Put in a wait here
            # Sleep for 1 second
            updatedTime = rospy.get_time()
            passedTime = updatedTime - driveStartTime

            # debug print statement
            #print('Updated: ' + str(passedTime))

            self._pub.publish(twist_message)
            rospy.sleep(0.05)


    # Rotate to a specific angle
    def rotate(self,angle):

        beginning_angle = self._yaw
        print('This is the beginning Angle: ' + str(beginning_angle))

        # Calculate the destination angle
        dest_angle = beginning_angle + angle

        dest_angle = (dest_angle % (2 * math.pi))

        if(dest_angle < 0):
            dest_angle = dest_angle + (2*math.pi)

        print('This is the dest_angle: ' + str(dest_angle))

        thresh = 0.1

        diff_angle = abs(dest_angle) - abs(self._yaw)

        print('This is the difference: ' + str(diff_angle))

        if( diff_angle > math.pi):
            rotateLeft = True
            diff_angle = -1

        if(diff_angle < -math.pi):
            rotateLeft = False
            diff_angle = 1

        # Find out which way to turn
        if(diff_angle < 0):
            # Angle is negative, turn clockwise

            diff_angle = abs(dest_angle) - abs(self._yaw)
            # Continually rotate until angle is reached
            while abs(diff_angle) > thresh and not self.interrupt:
                diff_angle = abs(dest_angle) - abs(self._yaw)
                print('This is the difference: ' + str(diff_angle))
                self.spinWheels(0.2,-0.2, 0.1)
            # Stop motors
            self.spinWheels(0,0,0.1)
        else:
            # Angle is positive, turn counter-clockwise
            diff_angle = dest_angle - self._yaw
            # Continuously rotate until angle is reached
            while abs(diff_angle) > thresh and not self.interrupt:
                diff_angle = dest_angle - self._yaw
                print('This is the difference: ' + str(diff_angle))
                self.spinWheels(-0.2,0.2, 0.1)
            # Stop motors
            self.spinWheels(0,0,0.1)

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

        self._yaw = yaw

    # helper functions
    def planTraj(self, b, t):
        """
            Bonus Question:  compute the coefs for a cubic polynomial (hint:  you did this in 3001)
        """

    def localizationSpin(self):
        print("Started the localization spin")
        spinTime = 20 # Time the robot spends spinning
        self.spinWheels(-0.5,0.5,spinTime)
        print("We have completed the localization spin")

if __name__ == '__main__':

    rospy.init_node('drive_base')

    listener = tf.TransformListener()
    turtle = Robot()

#    try:
#        # transform the nav goal from the global coordinate system to the robot's coordinate system
#        transGoal = self._odom_list.transformPose('/odometry', goal)
#        (trans,rot) = listener.lookupTransform(turtle, rospy.Time(0))
#    except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
#        pass

    #test function calls here
    rospy.Rate(10).sleep()

    flag = False
    while not rospy.is_shutdown():
        if not flag:
            turtle.localizationSpin()
            flag = True
            print("swv")
            turtle.spinWheels(0, 0, 0.1)

        # Prints for debugging
        print('X: ' + str(turtle._current.position.x))
        print('Y: ' + str(turtle._current.position.y))
        print('z: ' + str(turtle._yaw))

        rospy.sleep(1.)
        pass

    # Turn off motors
    turtle.interrupt = True
    turtle.spinWheels(0,0,1.)



