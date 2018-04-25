#!/usr/bin/env python

# File: Stop.py
# Purpose: Stop the robot from moving with one simple function
# Author: Clayton Dembski, Floris VanRossum
# Class: RBE3002
# Project: Final Project
# Professor: Professor Pinciroli


#####################IMPORTS##############################
import rospy, tf, copy, math, roslib, sys
from geometry_msgs.msg import Point, Twist, Pose, PoseStamped


#THIS IS A NODE. TREAT AS SUCH
##########################################################

def stopMoving():
    tempVelocityPublisher = rospy.Publisher('cmd_vel', Twist, None, queue_size = 1)

    zeroTwist = Twist()
    zeroTwist.linear.x = 0
    zeroTwist.angular.z = 0

    tempVelocityPublisher.publish(zeroTwist)
    print("\n\nStop Command Sent!")

if __name__ == '__main__':
    print("I am going to try to stop the robot from moving")
    stopMoving()