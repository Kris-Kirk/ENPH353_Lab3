#! /usr/bin/env python3

##This script is used subscribe to the topic /rrbot/camera1/image_raw, modify the input images and then publish a movement control Twist message to the topic /cmd_vel.

import rospy
import cv2
import numpy as np
from sensor_msgs.msg import Image
from geometry_msgs.msg import Twist
from cv_bridge import CvBridge, CvBridgeError

class image_converter:

    def __init__(self):
        self.bridge = CvBridge()
        self.image_sub = rospy.Subscriber('/rrbot/camera1/image_raw',Image,self.callback)
        self.cmd_vel_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
        self.twist = Twist()

    def callback(self,data):
        # try:
        #     cv_image = self.bridge.imgmsg_to_cv2(data, "bgr8")
        # except CvBridgeError as e:
        #     print(e)
        
        
        # ##Code to convert the image to a x and y coordinates
        
        # ## Blur the image with a 7x7 kernel with a 0 standard deviation.
        # #  0 is chosen to allow OpenCV to choose an appropriate deviation given
        # #  the Kernel size.
        # blur = cv2.GaussianBlur(data,(7,7),2)

        # ## Grayscales the blurred image to reduce the amount of data.
        # grayscale = cv2.cvtColor(blur, cv2.COLOR_BGR2GRAY)

        # ## Creates a black and white image by turning all pixels below a threshhold
        # #  to black. Anything over the 80 threshhold are set to 255. Less --> 0.
        # #  Makes the road white and the background black.
        # ret,blackWhite = cv2.threshold(grayscale,80,255,cv2.THRESH_BINARY_INV) ##INV is needed when contours is being used

        # ## Crops the screen to only look at the bottom 40 pixels of the image, to
        # #  Omit any of the road that might exist ahead. Limits the view to just the
        # #  bottom of the screen.
        # cropped = blackWhite[200:240, 1:320]

        # ## Finds the outermost contour of the image and returns the contour as a
        # #  list of points.

        # contours,hierarchy = cv2.findContours(cropped, 1, 2)

        # ## Only accepts the new contour if one was found. If no contour was found,
        # #  the previous contour is used for analysis.
        # if len(contours) != 0:
        #     cnt = contours[0] #Choose the first detected contour
        #     M = cv2.moments(cnt)


        # ## Was getting a divided by zero error, so made a special case to skip the
        # #  centroid calculation if M['m00'] which is the area of the shape
        # #  was 0, and just using the previous values of cx and cy.
        # if M['m00'] != 0:
        #     cx = int(M['m10']/M['m00'])
        #     cy = int(M['m01']/M['m00'])

        # ##Code to decide whether to move left, right or straight based on the x and y coordinates
        # rate = rospy.Rate(2) # 2hz
        # if cx < 120:
        #     self.twist.linear.x = 0.5
        #     self.twist.angular.z = 0.5
        # elif cx > 120:
        #     self.twist.linear.x = 0.5
        #     self.twist.angular.z = -0.5
        # else:
        #     self.twist.linear.x = 0.5
        #     self.twist.angular.z = 0.0

        move = Twist()
        move.linear.x = 0.5
        move.angular.z = 0.5

        ##Code to publish the Twist message
        try:
            self.cmd_vel_pub.publish(self.twist)
        except CvBridgeError as e:
            print(e)
def main():
    ic = image_converter()
    rospy.init_node('image_converter', anonymous=True)
    rospy.Subscriber("rrbot/camera1/image_raw",Image,ic.callback)
    try:
        rospy.spin()
    except KeyboardInterrupt:
        print("Shutting down")

if __name__ == '__main__':
    main()
