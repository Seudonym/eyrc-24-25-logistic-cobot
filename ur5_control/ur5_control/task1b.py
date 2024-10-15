#!/usr/bin/python3
# -*- coding: utf-8 -*-

'''
*****************************************************************************************
*
*        		===============================================
*           		    Logistic coBot (LB) Theme (eYRC 2024-25)
*        		===============================================
*
*  This script should be used to implement Task 1B of Logistic coBot (LB) Theme (eYRC 2024-25).
*
*  This software is made available on an "AS IS WHERE IS BASIS".
*  Licensee/end user indemnifies and will keep e-Yantra indemnified from
*  any and all claim(s) that emanate from the use of the Software or
*  breach of the terms of this agreement.
*
*****************************************************************************************
'''

# Team ID:          [ Team-ID ]
# Author List:		[ Names of team members worked on this file separated by Comma: Name1, Name2, ... ]
# Filename:		    task1b_boiler_plate.py
# Functions:
#			        [ Comma separated list of functions in this file ]
# Nodes:		    Add your publishing and subscribing node
#			        Publishing Topics  - [ /tf ]
#                   Subscribing Topics - [ /camera/aligned_depth_to_color/image_raw, /etc... ]


################### IMPORT MODULES #######################

import rclpy
from rclpy.node import Node
import sys
import cv2
import numpy as np
from cv_bridge import CvBridge
import tf2_ros
from geometry_msgs.msg import TransformStamped
from sensor_msgs.msg import Image
import math
from tf_transformations import quaternion_from_euler

##################### FUNCTION DEFINITIONS #######################

def calculate_rectangle_area(coordinates):
    '''
    Description:    Function to calculate area or detected aruco

    Args:
        coordinates (list):     coordinates of detected aruco (4 set of (x,y) coordinates)

    Returns:
        area        (float):    area of detected aruco
        width       (float):    width of detected aruco
    '''

    ############ Function VARIABLES ############

    area = None
    width = None

    ############ ADD YOUR CODE HERE ############

    # Ensure coordinates is a numpy array
    coordinates = np.array(coordinates)

    # Calculate the distances between consecutive points
    distances = np.sqrt(np.sum(np.diff(coordinates, axis=0)**2, axis=1))

    # The width is the average of the two shorter sides
    width = np.mean(sorted(distances)[:2])

    # Calculate the area using the shoelace formula
    x = coordinates[:, 0]
    y = coordinates[:, 1]
    area = 0.5 * np.abs(np.dot(x, np.roll(y, 1)) - np.dot(y, np.roll(x, 1)))

    ############################################

    return area, width

def detect_aruco(image):
    '''
    Description:    Function to perform aruco detection and return each detail of aruco detected 
                    such as marker ID, distance, angle, width, center point location, etc.

    Args:
        image                   (Image):    Input image frame received from respective camera topic

    Returns:
        center_aruco_list       (list):     Center points of all aruco markers detected
        distance_from_rgb_list  (list):     Distance value of each aruco markers detected from RGB camera
        angle_aruco_list        (list):     Angle of all pose estimated for aruco marker
        width_aruco_list        (list):     Width of all detected aruco markers
        ids                     (list):     List of all aruco marker IDs detected in a single frame 
    '''

    ############ Function VARIABLES ############

    aruco_area_threshold = 1500
    cam_mat = np.array([[931.1829833984375, 0.0, 640.0], [0.0, 931.1829833984375, 360.0], [0.0, 0.0, 1.0]])
    dist_mat = np.array([0.0,0.0,0.0,0.0,0.0])
    size_of_aruco_m = 0.15

    center_aruco_list = []
    distance_from_rgb_list = []
    angle_aruco_list = []
    width_aruco_list = []
    ids = []
 
    ############ ADD YOUR CODE HERE ############

    gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
    aruco_dict = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_50)
    parameters = cv2.aruco.DetectorParameters()
    detector = cv2.aruco.ArucoDetector(aruco_dict, parameters)
    
    corners, ids, _ = detector.detectMarkers(gray)

    if ids is not None:
        cv2.aruco.drawDetectedMarkers(image, corners, ids)
        
        for i, corner in enumerate(corners):
            area, width = calculate_rectangle_area(corner[0])
            if area > aruco_area_threshold:
                rvec, tvec, _ = cv2.aruco.estimatePoseSingleMarkers(corner, size_of_aruco_m, cam_mat, dist_mat)
                center = np.mean(corner[0], axis=0)
                distance = np.linalg.norm(tvec)
                angle = math.atan2(tvec[0][0][0], tvec[0][0][2])
                
                center_aruco_list.append(center)
                distance_from_rgb_list.append(distance)
                angle_aruco_list.append(angle)
                width_aruco_list.append(width)
                
                cv2.drawFrameAxes(image, cam_mat, dist_mat, rvec, tvec, 0.1)

    ############################################

    return center_aruco_list, distance_from_rgb_list, angle_aruco_list, width_aruco_list, ids

##################### CLASS DEFINITION #######################

class aruco_tf(Node):
    '''
    ___CLASS___

    Description:    Class which servers purpose to define process for detecting aruco marker and publishing tf on pose estimated.
    '''

    def __init__(self):
        '''
        Description:    Initialization of class aruco_tf
                        All classes have a function called __init__(), which is always executed when the class is being initiated.
                        The __init__() function is called automatically every time the class is being used to create a new object.
                        You can find more on this topic here -> https://www.w3schools.com/python/python_classes.asp
        '''

        super().__init__('aruco_tf_publisher')                                          # registering node

        ############ Topic SUBSCRIPTIONS ############

        self.color_cam_sub = self.create_subscription(Image, '/camera/color/image_raw', self.colorimagecb, 10)
        self.depth_cam_sub = self.create_subscription(Image, '/camera/aligned_depth_to_color/image_raw', self.depthimagecb, 10)

        ############ Constructor VARIABLES/OBJECTS ############

        image_processing_rate = 0.2                                                     # rate of time to process image (seconds)
        self.bridge = CvBridge()                                                        # initialise CvBridge object for image conversion
        self.tf_buffer = tf2_ros.buffer.Buffer()                                        # buffer time used for listening transforms
        self.listener = tf2_ros.TransformListener(self.tf_buffer, self)
        self.br = tf2_ros.TransformBroadcaster(self)                                    # object as transform broadcaster to send transform wrt some frame_id
        self.timer = self.create_timer(image_processing_rate, self.process_image)       # creating a timer based function which gets called on every 0.2 seconds (as defined by 'image_processing_rate' variable)
        
        self.cv_image = None                                                            # colour raw image variable (from colorimagecb())
        self.depth_image = None                                                         # depth image variable (from depthimagecb())


    def depthimagecb(self, data):
        '''
        Description:    Callback function for aligned depth camera topic. 
                        Use this function to receive image depth data and convert to CV2 image

        Args:
            data (Image):    Input depth image frame received from aligned depth camera topic

        Returns:
        '''

        ############ ADD YOUR CODE HERE ############

        try:
            self.depth_image = self.bridge.imgmsg_to_cv2(data, desired_encoding="passthrough")
        except Exception as e:
            self.get_logger().error(f'Error converting depth image: {str(e)}')

        ############################################


    def colorimagecb(self, data):
        '''
        Description:    Callback function for colour camera raw topic.
                        Use this function to receive raw image data and convert to CV2 image

        Args:
            data (Image):    Input coloured raw image frame received from image_raw camera topic

        Returns:
        '''

        ############ ADD YOUR CODE HERE ############

        try:
            self.cv_image = self.bridge.imgmsg_to_cv2(data, "bgr8")

        except Exception as e:
            self.get_logger().error(f'Error converting color image: {str(e)}')

        ############################################


    def process_image(self):
        '''
        Description:    Timer function used to detect aruco markers and publish tf on estimated poses.

        Args:
        Returns:
        '''

        ############ Function VARIABLES ############
        
        sizeCamX = 1280
        sizeCamY = 720
        centerCamX = 640 
        centerCamY = 360
        focalX = 931.1829833984375
        focalY = 931.1829833984375
            

        ############ ADD YOUR CODE HERE ############

        if self.cv_image is None or self.depth_image is None:
            return
        
        center_aruco_list, distance_from_rgb_list, angle_aruco_list, width_aruco_list, ids = detect_aruco(self.cv_image)
        
        if ids is not None:
            for i in range(len(ids)):
                angle_aruco = angle_aruco_list[i]
                angle_aruco = (0.788 * angle_aruco) - ((angle_aruco**2) / 3160)
                
                # Use euler2quat instead of quaternion_from_euler
                quat = quaternion_from_euler(0, 0, angle_aruco)
                
                cX, cY = center_aruco_list[i]
                depth = self.depth_image[int(cY), int(cX)] / 1000.0  # Convert mm to m
                
                x = depth * (sizeCamX - cX - centerCamX) / focalX
                y = depth * (sizeCamY - cY - centerCamY) / focalY
                z = depth
                
                cv2.circle(self.cv_image, (int(cX), int(cY)), 5, (0, 255, 0), -1)
                
                t_cam = TransformStamped()
                t_cam.header.stamp = self.get_clock().now().to_msg()
                t_cam.header.frame_id = 'camera_link'
                t_cam.child_frame_id = f'cam_{ids[i][0]}'
                t_cam.transform.translation.x = x
                t_cam.transform.translation.y = y
                t_cam.transform.translation.z = z
                t_cam.transform.rotation.x = quat[1]
                t_cam.transform.rotation.y = quat[2]
                t_cam.transform.rotation.z = quat[3]
                t_cam.transform.rotation.w = quat[0]
                self.br.sendTransform(t_cam)
                
                try:
                    t_base = self.tf_buffer.lookup_transform('base_link', f'cam_{ids[i][0]}', rclpy.time.Time())
                    t_obj = TransformStamped()
                    t_obj.header.stamp = self.get_clock().now().to_msg()
                    t_obj.header.frame_id = 'base_link'
                    t_obj.child_frame_id = f'obj_{ids[i][0]}'
                    t_obj.transform = t_base.transform
                    self.br.sendTransform(t_obj)
                except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException) as e:
                    self.get_logger().error(f'TF lookup failed: {str(e)}')
        
        cv2.imshow('Aruco Detection', self.cv_image)
        cv2.waitKey(1)
        ############################################


##################### FUNCTION DEFINITION #######################

def main():
    '''
    Description:    Main function which creates a ROS node and spin around for the aruco_tf class to perform it's task
    '''

    rclpy.init(args=sys.argv)                                       # initialisation

    node = rclpy.create_node('aruco_tf_process')                    # creating ROS node

    node.get_logger().info('Node created: Aruco tf process')        # logging information

    aruco_tf_class = aruco_tf()                                     # creating a new object for class 'aruco_tf'

    rclpy.spin(aruco_tf_class)                                      # spining on the object to make it alive in ROS 2 DDS

    aruco_tf_class.destroy_node()                                   # destroy node after spin ends

    rclpy.shutdown()                                                # shutdown process


if __name__ == '__main__':
    '''
    Description:    If the python interpreter is running that module (the source file) as the main program, 
                    it sets the special __name__ variable to have a value "__main__". 
                    If this file is being imported from another module, __name__ will be set to the module's name.
                    You can find more on this here -> https://www.geeksforgeeks.org/what-does-the-if-__name__-__main__-do/
    '''

    main()