#!/usr/bin/env python3

"""
    moveit_cartesian_path.py - Version 0.1 2016-07-28

    Based on the R. Patrick Goebel's moveit_cartesian_demo.py demo code.

    Plan and execute a Cartesian path for the end-effector.

    Created for the Pi Robot Project: http://www.pirobot.org
    Copyright (c) 2014 Patrick Goebel.  All rights reserved.
    This program is free software; you can redistribute it and/or modify
    it under the terms of the GNU General Public License as published by
    the Free Software Foundation; either version 2 of the License, or
    (at your option) any later version.5

    This program is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU General Public License for more details at:

    http://www.gnu.org/licenses/gpl.html
"""

import rospy, sys, numpy as np
import moveit_commander
from copy import deepcopy
import geometry_msgs.msg
from ur5_notebook.msg import Tracker
import moveit_msgs.msg
import cv2, cv_bridge
from sensor_msgs.msg import Image


from std_msgs.msg import Header
from trajectory_msgs.msg import JointTrajectory
from trajectory_msgs.msg import JointTrajectoryPoint
tracker = Tracker()
tracker.blockColor = 0
class ur5_vision:
    def __init__(self):
        rospy.init_node("ur5_vision", anonymous=False)
        self.track_flag = False
        self.default_pose_flag = True
        self.cx = 400.0
        self.cy = 400.0
        self.bridge = cv_bridge.CvBridge()
        self.image_sub = rospy.Subscriber('/ur5/usbcam/image_raw', Image, self.image_callback)
        self.cxy_pub = rospy.Publisher('cxy', Tracker, queue_size=1)


    def image_callback(self,msg):
        # BEGIN BRIDGE
        image = self.bridge.imgmsg_to_cv2(msg,desired_encoding='bgr8')
        # END BRIDGE
        # BEGIN HSV
        hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)


        color_flags = []

        # END HSV
        # BEGIN FILTER
        # Creates a mask by thresholding for different shades of red
        lower_red = np.array([ 0,  100, 100])
        upper_red = np.array([10, 255, 255])
        mask_red = cv2.inRange(hsv, lower_red, upper_red)
        # Finds contours for the red region
        red_cnts, _ = cv2.findContours(mask_red.copy(), cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        # For each contour, append the red color flag (a float with a value of 0)
        for c in red_cnts:
           color_flags.append(0)

        # Creates a mask by thresholding for different shades of yellow
        lower_yellow = np.array([20, 100, 100])
        upper_yellow = np.array([30, 255, 255])
        mask_yellow = cv2.inRange(hsv, lower_yellow, upper_yellow)
        # Finds contours for the yellow region
        yellow_cnts, _ = cv2.findContours(mask_yellow.copy(), cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        # For each contour, append the yellow color flag (float of value 1)
        for c in yellow_cnts:
           color_flags.append(1)

        # Creates a mask by thresholding for different shades of blue
        lower_blue = np.array([100, 150, 0])
        upper_blue = np.array([ 140,  255, 255])
        mask_blue = cv2.inRange(hsv, lower_blue, upper_blue)
        # Find contours for the blue region
        blue_cnts, _ = cv2.findContours(mask_blue.copy(), cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        # For each contour, append the blue color flag (float of value 2)
        for c in blue_cnts:
           color_flags.append(2)

        # Find contours in the binary masked image
        # 	Params: mask to find contours in , mode: look for extenal contours, method: approximate using end points
        
        h, w, d = image.shape
        # print h, w, d  (800,800,3)

        #BEGIN FINDER
        # Finds the center of the masked region
        # 	Moments are a weighted average of each frame in your image
        # 	m00 is the total area of the masked region.
        # 	m10 is the non normalized expected value of the masked x region
        # 	m01 is the non normalized expected value of the masked y region
        red_moments = cv2.moments(mask_red)
        if red_moments['m00'] > 0:
            red_cx = int(red_moments['m10']/red_moments['m00'])
            red_cy = int(red_moments['m01']/red_moments['m00'])

        yellow_moments = cv2.moments(mask_yellow)
        if yellow_moments['m00'] > 0:
            yellow_cx = int(yellow_moments['m10']/yellow_moments['m00'])
            yellow_cy = int(yellow_moments['m01']/yellow_moments['m00'])

        blue_moments = cv2.moments(mask_blue)
        if blue_moments['m00'] > 0:
            blue_cx = int(blue_moments['m10']/blue_moments['m00'])
            blue_cy = int(blue_moments['m01']/blue_moments['m00'])
        # cx range (55,750) cy range( 55, ~ )
        # END FINDER

        # Isolate largest contour
        # Append all the contours in a list
        cnts = red_cnts + yellow_cnts + blue_cnts


        contour_sizes = [(cv2.contourArea(contour), contour, color_flags[idx]) for idx, contour in enumerate(cnts)]
        
        # If any contours were found
        if(len(cnts) > 0):
            # give us the largest contour (maximizing area) and some meta data... [area of contour, contour, color]
            biggest_res = max(contour_sizes, key=lambda x: x[0])
        else: # Default the area to be 0, the contour to be 0, and the color to be 0 (red)
            biggest_res = [0,0,0]

        biggest_contour_area = biggest_res[0]
        biggest_contour = biggest_res[1]
        biggest_contour_color = biggest_res[2]

        # If the area is large enough to be a block, load data into the Tracker message to be published with ROS
        if biggest_contour_area > 7500:
            # Depending on the color of the largest contour, set cx and cy equal the the cx and cy of the centroid of that mask
            if biggest_contour_color == 0:
                cx = red_cx
                cy = red_cy
            elif biggest_contour_color == 1:
                cx = yellow_cx
                cy = yellow_cy
            elif biggest_contour_color == 2:
                cx = blue_cx
                cy = blue_cy
            
            # Initialize the message to be published
            self.track_flag = True
            self.cx = cx
            self.cy = cy
            self.error_x = self.cx - w/2
            self.error_y = self.cy - (h/2+195)
            tracker.x = cx
            tracker.y = cy
            tracker.flag1 = self.track_flag
            tracker.error_x = self.error_x
            tracker.error_y = self.error_y
            tracker.blockColor = biggest_contour_color # 0 if red, 1 if yellow, 2 if blue

            # BEGIN circle
            #	 Draw a dot at the center of the masked region
            cv2.circle(image, (cx, cy), 10, (0,0,0), -1)
            cv2.putText(image, "({}, {})".format(int(cx), int(cy)), (int(cx-5), int(cy+15)), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 1)
            cv2.drawContours(image, cnts, -1, (255, 255, 255),1)
            #BGIN CONTROL
        else:
            self.track_flag = False
            tracker.flag1 = self.track_flag
            tracker.blockColor = -1

        self.cxy_pub.publish(tracker)
        cv2.namedWindow("window", 1)
        cv2.imshow("window", image )
        cv2.waitKey(1)

follower=ur5_vision()
rospy.spin()
