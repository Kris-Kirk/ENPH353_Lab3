#! /usr/bin/env python3

##This script is used subscribe to the topic /rrbot/camera1/image_raw, modify the input images and then publish a movement control Twist message to the topic /cmd_vel.

import rospy
import cv2
import numpy as np
from sensor_msgs.msg import Image
from geometry_msgs.msg import Twist
from cv_bridge import CvBridge, CvBridgeError
from sensor_msgs.msg import Image

class image_converter:

    def __init__(self):
        self.bridge = CvBridge()
        self.image_sub = rospy.Subscriber('/rrbot/camera1/image_raw',Image,self.callback)
        self.cmd_vel_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
        self.twist = Twist()
        self.prev_cx = 400

    def callback(self,data):
        try:
            cv_image = self.bridge.imgmsg_to_cv2(data)
        except CvBridgeError as e:
            print(e)
        

        ##Code to convert the image to a x and y coordinates
        
        ## Blur the image with a 7x7 kernel with a 0 standard deviation.
        #  0 is chosen to allow OpenCV to choose an appropriate deviation given
        #  the Kernel size.
        blur = cv2.GaussianBlur(cv_image,(7,7),4)

        ## Grayscales the blurred image to reduce the amount of data.
        grayscale = cv2.cvtColor(blur, cv2.COLOR_BGR2GRAY)

        ## Creates a black and white image by turning all pixels below a threshhold
        #  to black. Anything over the 80 threshhold are set to 255. Less --> 0.
        #  Makes the road white and the background black.
        ret,blackWhite = cv2.threshold(grayscale,80,255,cv2.THRESH_BINARY_INV) ##INV is needed when contours is being used

        ## Crops the screen to only look at the bottom 40 pixels of the image, to
        #  Omit any of the road that might exist ahead. Limits the view to just the
        #  bottom of the screen.
        cropped = blackWhite[700:800, 1:800]
        finalImg = cropped
        ## Finds the outermost contour of the image and returns the contour as a
        #  list of points.

        contours,hierarchy = cv2.findContours(cropped, 1, 2)

        ## Only accepts the new contour if one was found. If no contour was found,
        #  the previous contour is used for analysis.

        worked = False

        if len(contours) != 0:
            cnt = contours[0] #Choose the first detected contour
            M = cv2.moments(cnt)
            # print("contour found")

            ## Was getting a divided by zero error, so made a special case to skip the
            #  centroid calculation if M['m00'] which is the area of the shape
            #  was 0, and just using the previous values of cx and cy.
            if M['m00'] != 0:
                cx = int(M['m10']/M['m00'])
                self.prev_cx = cx
                worked = True
                # print(cx)
                ##Code to decide whether to move left, right or straight based on the x and y coordinates
                self.twist = image_converter.steer(cx)
                copy = cv_image.copy()
                finalImg = cv2.circle(copy,[cx,750],10,[255,255,0],cv2.FILLED)
        
        if worked == False:
            self.twist = image_converter.steer(self.prev_cx)
            
        ##Code to publish the Twist message
        try:
            self.cmd_vel_pub.publish(self.twist)
        except CvBridgeError as e:
            print(e)

    def steer(cx):
        newTwist = Twist()
        if cx < 350:
            newTwist.linear.x = 0.07
            newTwist.angular.z = 3
        elif cx > 450:
            newTwist.linear.x = 0.07
            newTwist.angular.z = -3
        else:
            newTwist.linear.x = 0.1
            newTwist.angular.z = 0.0
        return newTwist

def main():
    rospy.init_node('image_converter', anonymous=True)
    ic = image_converter()
    rospy.Subscriber("rrbot/camera1/image_raw",Image,ic.callback)
    try:
        rospy.spin()
    except KeyboardInterrupt:
        print("Shutting down")

if __name__ == '__main__':
    main()