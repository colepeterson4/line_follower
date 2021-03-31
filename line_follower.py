#!/usr/bin/env python
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import rospy
from geometry_msgs.msg import Twist
import numpy as np


rospy.init_node('line_follower')
BRIDGE = CvBridge()

def cv_callback(msg):
    #converting to a cv image from ROS.
    cv_image = CvBridge().imgmsg_to_cv2(msg, "bgr8")

    #getting the dimensions of the image (we won't actually use the color given by shape).
    height, width, _ = cv_image.shape

    #cropping the image so we only see the bottom.
    cv_image = cv_image[height-200:height, 50:width-50]

    #dimensions have changed
    height, width, _ = cv_image.shape

    #The image was too large, causing slow calculations later on. this shrinks it down.
    cv_image = cv2.resize(cv_image, (height/4, width/4), interpolation = cv2.INTER_AREA)

    #sending the image off to the color detection function
    color(cv_image)

#takes an image and averges the horizontal position of any yellow pixels
def color(cv_img):
    global error
    height, width, _ = cv_img.shape

    img = cv2.cvtColor(cv_img, cv2.COLOR_BGR2HSV)
    #variable which stores the sum of all horizontal positions of yellow pixels
    sum_of_positions = 0.0
    #variable which stores the count of yellow pixels
    total_pixels = 1.0

    lower_yellow = np.array([20, 100, 100])            # lower color threshold
    upper_yellow = np.array([30, 255, 255])          # upper color threshold
 
    # calculate a mask to impose on the target image
    mask = cv2.inRange(img, lower_yellow, upper_yellow)

    result = cv2.bitwise_and(img, img, mask = mask)


    for i in range(0, height):
        for j in range(0, (width)):
            if result[i][j][0] > 0:
                sum_of_positions += j
                total_pixels += 1
    if total_pixels == 1.0: 
        print("NO YELLOW PIXELS")
        error = 0
    else:  
        error = (sum_of_positions / total_pixels) - width/2

cam_sub = rospy.Subscriber('/camera/rgb/image_raw', Image, cv_callback)
rate = rospy.Rate(10)
pub = rospy.Publisher('cmd_vel', Twist, queue_size = 1)
twist = Twist()
twist.linear.x = 0.2 
error = 0
while not rospy.is_shutdown():
    if error > 1:
        twist.angular.z = -0.2
    elif error < 1:
        twist.angular.z = 0.2
    else:
        twist.angular.z = 0
    pub.publish(twist)
    rate.sleep()