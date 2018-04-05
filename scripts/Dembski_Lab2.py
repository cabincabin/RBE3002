#!/usr/bin/env python
#Clayton Dembski
#RBE3002
#Lab 2 Code
import rospy, tf, copy, math

from geometry_msgs.msg import Twist, Pose, PoseStamped
from tf.transformations import euler_from_quaternion


import numpy as np
from std_msgs.msg import String

class Robot:
    def __init__(self):

        """
            This constructor sets up class variables and pubs/subs
        """
        self._current = Pose() # initlize correctly
        self._odom_list = tf.TransformListener()
        #Update the position of the robot every .1 seconds
        rospy.Timer(rospy.Duration(.1), self.timerCallback)
        #USE '/cmd_vel" for robot and '/cmd_vel_mux/input/teleop' for gazebo.
        #Publishes Velocity Updates
        self._vel_pub = rospy.Publisher('/cmd_vel', Twist, None, queue_size=1)#'/cmd_vel_mux/input/teleop'
        #recive RVIZ callback for 2DNAVGOAL, call NavToPose
        rospy.Subscriber('/move_base_simple/goal', PoseStamped, self.navToPose, queue_size=1) # handle nav goal events


    def navToPose(self, goal):
        """
            This is a callback function. It should exract data from goal, drive in a striaght line to reach the goal and
            then spin to match the goal orientation.
        """

        currpose = copy.deepcopy(self._current)

        self._odom_list.waitForTransform('/odom', '/base_footprint', rospy.Time(0), rospy.Duration(1.0))
        #realtime wait for use with robot, means that time stamps happen in order
        rospy.Rate(1).sleep()
        rospy.Rate(1).sleep()

        #get the goal position in the robot coordinate space, instead of RVIZ
        transGoal = self._odom_list.transformPose('/odom', goal)#may need to be an odom

        #see current position and end position
        # print("TRANSGOAL")
        # print(transGoal.pose.position.x)
        # print(transGoal.pose.position.y)
        # print(transGoal.pose.position.z)
        # print("CURRENT")
        # print(currpose.position.x)
        # print(currpose.position.y)
        # print(currpose.position.z)

        # # transform the nav goal from the global coordinate system to the robot's coordinate system
        dir = math.atan2(transGoal.pose.position.y-currpose.position.y, transGoal.pose.position.x-currpose.position.x)
        #convert to 0 to 2 pi
        if dir<0:
            dir+2*math.pi

        #get the angle from the robot to the goal
        dist = math.sqrt((currpose.position.x-transGoal.pose.position.x)*(currpose.position.x-transGoal.pose.position.x)
            + (transGoal.pose.position.y-currpose.position.y)*(transGoal.pose.position.y-currpose.position.y))
        # #ros PosePostiton, oriantation

        # print("dist")
        # print(dist)
        #
        # print("angle")
        # print(dir*180/math.pi)
        q1 = [self._current.orientation.x,
              self._current.orientation.y,
              self._current.orientation.z,
              self._current.orientation.w]

        # convert the quaternion to roll pitch yaw
        (roll1, pitch1, yaw1) = euler_from_quaternion(q1)

        #turn to face the point
        self.rotate(dir-yaw1)

        #drive till at that point
        self.driveStraight(.1, dist)

        q1 = [self._current.orientation.x,
              self._current.orientation.y,
              self._current.orientation.z,
              self._current.orientation.w]

        # convert the quaternion to roll pitch yaw
        (roll1, pitch1, yaw1) = euler_from_quaternion(q1)

        q2 = [transGoal.pose.orientation.x,
              transGoal.pose.orientation.y,
              transGoal.pose.orientation.z,
              transGoal.pose.orientation.w]

        # convert the quaternion to roll pitch yaw
        (roll2, pitch2, yaw2) = euler_from_quaternion(q2)
        rospy.Rate(1).sleep()
        if yaw1 < 0:
            yaw1 = yaw1 + 2 * math.pi

        if yaw2 < 0:
            yaw2 = yaw2 + 2 * math.pi


        print("angle")
        print(yaw1)
        print(yaw2)
        print(yaw2-yaw1)

        #rotate till facing thr right way
        self.rotate(yaw2-yaw1)

        #stop motion at end
        twist = Twist()
        self._vel_pub.publish(twist)



    def executeTrajectory(self):
        """
            See lab manual for the dance the robot has to excute
        """

        #drive straight for 60cm
        self.driveStraight(.1, .6)
        rospy.Rate(1).sleep()
        #rotate 90 degrees to right
        self.rotate(math.pi/2)
        rospy.Rate(1).sleep()
        #drive straight for 45 cm
        self.driveStraight(.1, .45)
        rospy.Rate(1).sleep()
        #rotate 135 degrees left
        self.rotate(-3*math.pi/4)


    def driveStraight(self, speed, distance):
        """
            This method should populate a ??? message type and publish it to ??? in order to move the robot
        """
        #Dead Recon. This is only useful for very short distances. End position should be checked and updated afterwords
        self.spinWheels(speed,speed,(float(distance)/float(speed)))

    def spinWheels(self, v_left, v_right, time):
        """
           This method should use differential drive kinematics to compute V and omega (linear x = V, angular z = omega).
           It should then create a ??? message type, and publish it to ??? in order to move the robot
        """
        #this would cause a divide by 0 error, hense if else
        if  (v_left < v_right+.0001) and (v_left > v_right-.0001):
            twist = Twist()
            twist.linear.x = v_left
            twist.angular.z = 0
            driveStartTime = rospy.Time.now().secs

            # drive in a straight line
            # print(driveStartTime)
            # print(rospy.Time.now().secs)
            while(rospy.Time.now().secs <= driveStartTime+time):
                #print(rospy.Time.now().secs)
                self._vel_pub.publish(twist)
                rospy.Rate(10).sleep()



        else:
            diameter = 0.23 # based on wheel track from https://yujinrobot.github.io/kobuki/doxygen/enAppendixKobukiParameters.html

            #calculate angular velocity
            omega = (v_right - v_left)/diameter
            r = diameter/2 * (v_right+v_left)/(v_right-v_left)

            v = omega*r

            #calculate actual velocity
            driveStartTime = rospy.Time.now().secs
            twist = Twist()

            twist.linear.x = v
            twist.angular.z = omega

            #Drive in an arc
            self._vel_pub.publish(twist)
            print(driveStartTime)
            print(rospy.Time.now().secs)

            while (rospy.Time.now().secs <= driveStartTime + time):
                #print(rospy.Time.now().secs)
                self._vel_pub.publish(twist)
                rospy.Rate(10).sleep()
        # stop motion at end
        twist = Twist()
        self._vel_pub.publish(twist)




    def rotate(self,angle):
    #     """
    #         This method should populate a ??? message type and publish it to ??? in order to spin the robot
    #     """

        self._odom_list.waitForTransform('/odom', '/base_footprint', rospy.Time(0), rospy.Duration(1.0))


        q = [self._current.orientation.x,
             self._current.orientation.y,
             self._current.orientation.z,
             self._current.orientation.w]

        # convert the quaternion to roll pitch yaw
        #get the current position/pose of robot
        (roll, pitch, yaw) = euler_from_quaternion(q)

        #get the global angle that the robot needs to be at
        goalAng = (yaw + angle)%(2*math.pi)

        if goalAng < 0:
            goalAng + 2*math.pi

        if yaw < 0:
            yaw = yaw+2*math.pi

        #convert from +- pi to 0 to 2 pi


        #change the way the robot rotates based on what angle is closer
        if angle>0:
            mult = -1
        else:
            mult = 1

        while (yaw > goalAng+.1) or (yaw < goalAng-.1):
            #spin while not within a thresh of the angle
            self.spinWheels(mult*.01, mult*-.01, .1)
            # print("..................")
            # print(goalAng)
            # print(yaw)
            # print("..................")

            #update current angle info
            q = [self._current.orientation.x,
                 self._current.orientation.y,
                 self._current.orientation.z,
                 self._current.orientation.w]

            (roll, pitch, yaw) = euler_from_quaternion(q)

            if (yaw < 0):
                yaw = yaw + 2 * math.pi
        # print("DONE")
        # stop motion at end
        twist = Twist()
        self._vel_pub.publish(twist)


    #
    #
    def timerCallback(self,evprent):
        """
            This is a callback that runs every 0.1s.
            Updates this instance of Robot's internal position variable (self._current)
        """
    # wait for and get the transform between two frames
        #get transform to base footprint
        self._odom_list.waitForTransform('/odom', '/base_footprint', rospy.Time(0), rospy.Duration(1.0))

        (position, orientation) = self._odom_list.lookupTransform('/odom','/base_footprint', rospy.Time(0))
    # save the current position and orientation

        #update class info for robot position
        self._current.position.x = position[0]
        self._current.position.y = position[1]
        self._current.orientation.x = orientation[0]
        self._current.orientation.y = orientation[1]
        self._current.orientation.z = orientation[2]
        self._current.orientation.w = orientation[3]


if __name__ == '__main__':

    rospy.init_node('drive_base')
    turtle = Robot()
    rospy.Time.now().secs
    #test function calls here
    bool1 = True
    while not rospy.is_shutdown():
        rospy.Rate(10).sleep()
        if bool1 == False:
            #turtle.spinWheels(1, 1, 5)
            rospy.Rate(10).sleep()
            # turtle.driveStraight(.1,1)
            bool1 = True
        pass
