#!/usr/bin/env/python

'''
    File Name: Main.py
    Author: Adam Walker
    Date Created: 14/03/2017
    Date Last Modified: 16/03/2017
    Python Version: 2.7

'''

import rospy, cv2, cv_bridge, numpy
from sensor_msgs.msg import Image
from geometry_msgs.msg import Twist

class Follower:

    def __init__(self):
        self.bridge = cv_bridge.CvBridge()
        cv2.namedWindow("window", 1)
        cv2.namedWindow("mask", 1)
        self.image_sub = rospy.Subscriber('/turtlebot/camera/rgb/image_raw',
                                          Image, self.image_callback)
        self.cmd_vel_pub = rospy.Publisher('/turtlebot/cmd_vel',
                                           Twist, queue_size=1)
        self.twist = Twist()

        self.velocity = 0.5
        self.colourTarget = 0
        self.colourFound = [0,0,0,0]

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
            lower_bound = numpy.array([ 100, 50,  50])
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

        if M['m00'] > 0:
            cx = int(M['m10']/M['m00'])
            cy = int(M['m01']/M['m00'])
            cv2.circle(image, (cx, cy), 20, (0,0,255), -1)

            if(self.colourTarget == 0):
                self.colourFound[0] = 1
                self.colourTarget = (self.colourTarget+1)%4
                print("Yellow Found")

            elif(self.colourTarget == 1):
                self.colourFound[1] = 1
                self.colourTarget = (self.colourTarget+1)%4
                print("Green Found")

            elif(self.colourTarget == 2):
                self.colourFound[2] = 1
                self.colourTarget = (self.colourTarget+1)%4
                print("Blue Found")

            elif(self.colourTarget == 3):
                self.colourFound[3] = 1
                self.colourTarget = (self.colourTarget+1)%4
                print("Red Found")

            # BEGIN CONTROL
            err = cx - w/2
            self.twist.linear.x = self.velocity
            self.twist.angular.z = -float(err) / 100
            self.cmd_vel_pub.publish(self.twist)
            # END CONTROL
        else:
            self.colourTarget = (self.colourTarget+1)%4
        cv2.imshow("window", image)
        cv2.imshow("mask", mask)
        cv2.waitKey(1)

rospy.init_node('follower')
follower = Follower()
rospy.spin()
