#!/usr/bin/env/python

'''
    File Name: Main.py
    Author: Adam Walker
    Date Created: 14/03/2017
    Date Last Modified: 22/03/2017
    Python Version: 2.7
'''

import rospy, cv2, cv_bridge, numpy, math
from sensor_msgs.msg import Image
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan
from math import radians
from numpy import nanmean, nanmin, nansum

class Follower:

    def __init__(self):
        self.bridge = cv_bridge.CvBridge()
        cv2.namedWindow("window", 1)
        cv2.namedWindow("mask", 1)
        self.image_sub = rospy.Subscriber('/turtlebot/camera/rgb/image_raw',
                                          Image, self.image_callback)

        self.laser_sub = rospy.Subscriber('/turtlebot/scan',
                                          LaserScan, self.laser_callback)

        self.cmd_vel_pub = rospy.Publisher('/turtlebot/cmd_vel',
                                           Twist, queue_size=1)

        self.twist = Twist()

        self.velocity = 1.2
        self.colourTarget = 0
        self.colourFound = [0,0,0,0]
        self.currentDist = 100
        self.laserArray = [0]
        self.leftQuart = len(self.laserArray) / 4
        self.rightQuart = self.leftQuart * 3

    def moveBot(self, linear, angular):
        self.twist.linear.x = linear
        self.twist.angular.z = angular
        self.cmd_vel_pub.publish(self.twist)

    def avoidance(self):

        if (math.isnan(nanmean(self.laserArray)) == False):
            if nanmean(self.laserArray) < 1.5:
                self.moveBot(0, 0.75)
                print("Front blocked - Turning")
            else:
                if (self.laserArray[1]) < (self.laserArray[(len(self.laserArray) - 1)]):
                    self.moveBot(0, 0.5)
                    print("Right Blocked - Going Left")
                elif (self.laserArray[(len(self.laserArray) - 1)]) < (self.laserArray[1]):
                    self.moveBot(0, -0.5)
                    print("Left Blocked - Going Right")
        else:
            self.moveBot(-1, 0)

    def image_callback(self, msg):
        image = self.bridge.imgmsg_to_cv2(msg,desired_encoding='bgr8')
        hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)

        # Yellow
        if( self.colourTarget == 0 and self.colourFound[0] == 0):
            lower_bound = numpy.array([ 30, 200,  100])
            upper_bound = numpy.array([ 50, 255, 150])
        # Green
        elif( self.colourTarget == 1 and self.colourFound[1] == 0):
            lower_bound = numpy.array([ 40, 100,  80])
            upper_bound = numpy.array([ 80, 255, 250])
        # Blue
        elif( self.colourTarget == 2 and self.colourFound[2] == 0):
            lower_bound = numpy.array([ 100, 100,  100])
            upper_bound = numpy.array([ 130, 255, 255])
        # Red
        elif( self.colourTarget == 3 and self.colourFound[3] == 0):
            lower_bound = numpy.array([ 0, 200, 100])
            upper_bound = numpy.array([ 0, 255, 150])
        # Otherwise just set the mask to white
        else:
            lower_bound = numpy.array([ 255, 255, 255])
            upper_bound = numpy.array([ 255, 255, 255])

        mask = cv2.inRange(hsv, lower_bound, upper_bound)

        h, w, d = image.shape

        M = cv2.moments(mask)

        if nanmin(self.laserArray > 0.75):
            print("Path 1")
            if M['m00'] > 0:
                cx = int(M['m10']/M['m00'])
                cy = int(M['m01']/M['m00'])
                cv2.circle(image, (cx, cy), 20, (0,0,255), -1)

                err = cx - w/2
                self.moveBot(self.velocity, -float(err) / 100)

                if(self.currentDist < 1.25) and (self.currentDist >= 0.75):
                    print("Path 2")
                    if(self.colourTarget == 0):
                        self.colourFound[0] = 1
                        self.colourTarget = (self.colourTarget + 1) % 4
                        print("============== Yellow Found ==============")

                    elif(self.colourTarget == 1):
                        self.colourFound[1] = 1
                        self.colourTarget = (self.colourTarget + 1) % 4
                        print("============== Green Found ==============")

                    elif(self.colourTarget == 2):
                        self.colourFound[2] = 1
                        self.colourTarget = (self.colourTarget + 1) % 4
                        print("============== Blue Found ==============")

                    elif(self.colourTarget == 3):
                        self.colourFound[3] = 1
                        self.colourTarget = (self.colourTarget + 1) % 4
                        print("============== Red Found ==============")

            else:
                self.colourTarget = (self.colourTarget + 1) % 4
                if nanmin(self.laserArray) < 1 and (math.isnan(self.currentDist) == False):
                    self.avoidance()
                    print("Path 3")
                else:
                    self.moveBot(self.velocity, 0)
                    print("Path 4")
        else:
            self.moveBot(0, -1)
            self.avoidance()
            print("Path 5")



        cv2.imshow("window", image)
        cv2.imshow("mask", mask)
        cv2.waitKey(1)

    def laser_callback(self, msg):
        self.currentDist = msg.ranges[len(msg.ranges) / 2]
        self.laserArray = msg.ranges

rospy.init_node('follower')
follower = Follower()
rospy.spin()
