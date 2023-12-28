#!/usr/bin/env python3

import rospy
from std_msgs.msg import Bool, String
from sensor_msgs.msg import Image  # Import the Image message type
from cv_bridge import CvBridge, CvBridgeError  # Import CvBridge
import cv2
from classification import SVM, getLabel
from utils import *
import os
import rospkg

# Initialize global variables
start_detection = False
current_image = None  # This will hold the latest image received from the camera
bridge = CvBridge()  # Instantiate a CvBridge object

def callback_start_detection(data):
    global start_detection
    start_detection = data.data

def callback_image(data):
    global current_image
    try:
        # Convert the ROS Image message to an OpenCV image
        current_image = bridge.imgmsg_to_cv2(data, "bgr8")
    except CvBridgeError as e:
        rospy.logerr(e)

def detect_signs(model, display_results, image):
    if image is None:
        return "No Image"

    coordinate, sign_type, text = localization(image, 300, 0.65, model)
    # Optionally display the results based on the ROS argument
    if display_results:
        processed_image = draw_picture(image, coordinate, sign_type, text)
        cv2.imwrite('detection.png', processed_image)
        cv2.imshow('Detection', processed_image)
        # cv2.waitKey(1)
    return text

def main():
    global model, pub, sub

    # Load the trained SVM model
    rospack = rospkg.RosPack()

    model = cv2.ml.SVM_load(rospack.get_path('mobile_robot')+"/src/data_svm.dat")

    # Initialize ROS node
    rospy.init_node('traffic_sign_detector', anonymous=True)

    # ROS Publisher and Subscriber
    pub = rospy.Publisher('traffic_sign', String, queue_size=10)
    sub = rospy.Subscriber('start_detection', Bool, callback_start_detection)

    # ROS Subscriber for RPi Camera
    sub_from_rpi_cam = rospy.Subscriber('/raspicam_node/image/compressed', Image, callback_image)

    # Set the rate of the loop
    rate = rospy.Rate(10)  # 10hz

    rospy.sleep(1)

    display_results = rospy.get_param("~display", False)

    while not rospy.is_shutdown():
        if start_detection and current_image is not None:
            most_common_sign = detect_signs(model, display_results, current_image)
            pub.publish(most_common_sign)
            rospy.loginfo(f'Published Most Common Traffic Sign: {most_common_sign}')
        else:
            pub.publish('WAITING FOR DETECT')

        rate.sleep()

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
