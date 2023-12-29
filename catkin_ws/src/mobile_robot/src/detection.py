#!/usr/bin/env python3

import rospy
from std_msgs.msg import Bool, String
from sensor_msgs.msg import CompressedImage  # Import the CompressedImage message type
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

SIGNS = ["STOP",
        "TURN LEFT",
        "TURN RIGHT",
        "ONE WAY"]

def callback_start_detection(data):
    global start_detection
    start_detection = data.data

def callback_image(data):
    global current_image
    try:
        # Convert the ROS CompressedImage message to an OpenCV image
        # np_arr = np.fromstring(data.data, np.uint8)
        np_arr = np.frombuffer(data.data, np.uint8)
        
        current_image = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)
    except CvBridgeError as e:
        rospy.logerr(e)

def detect_signs(model, image):
    if image is None:
        return "No Image"
    
    rospack = rospkg.RosPack()
    cv2.imwrite(rospack.get_path('mobile_robot')+"/src/original.png", image)
    #cv2.imshow('original', image)
    
    coordinate, _, sign_type, text = localization(image, 300, 0.65, model)
    
    # Optionally display the results based on the ROS argument
    # if text in SIGNS:
    if True:
        processed_image = draw_picture(image, coordinate, sign_type, text)
        #cv2.imwrite(rospack.get_path('mobile_robot')+f"/src/{text}.png", processed_image)
        cv2.imwrite(rospack.get_path('mobile_robot')+f"/src/detection.png", processed_image)
        # cv2.imshow('Detection', processed_image)
        # cv2.waitKey(1)
    return coordinate, _, sign_type, text

def main():
    global model, pub, sub
    global start_detection, current_image

    # Load the trained SVM model
    rospack = rospkg.RosPack()

    #model = cv2.ml.SVM_load(rospack.get_path('mobile_robot')+"/src/data_svm.dat")
    model = training()

    # Initialize ROS node
    rospy.init_node('traffic_sign_detector', anonymous=True)

    # ROS Publisher and Subscriber
    pub = rospy.Publisher('traffic_sign', String, queue_size=10)
    sub = rospy.Subscriber('start_detection', Bool, callback_start_detection)

    # ROS Subscriber for RPi Camera (using CompressedImage)
    sub_from_rpi_cam = rospy.Subscriber('raspicam_node/image/compressed', CompressedImage, callback_image)

    # Set the rate of the loop
    rate = rospy.Rate(10)  # 10hz

    rospy.sleep(1)

    count = 48
    
    sign = 3
    dir = rospack.get_path('mobile_robot')+f"/src/dataset/{sign}"
    os.makedirs(dir, exist_ok=True)

    while not rospy.is_shutdown():
        if start_detection and current_image is not None:
            coordinate, _, sign_type, text = detect_signs(model, current_image)
            pub.publish(text)
            rospy.loginfo(f'Published Traffic Sign: {text}')

            if False:
                print('coordinate', coordinate)
                if coordinate is not None:
                    top = int(coordinate[0][1])
                    left = int(coordinate[0][0])
                    bottom = int(coordinate[1][1])
                    right = int(coordinate[1][0])
                    # Crop the current_image with the contour bbox (top, left, bottom, right)
                    cropped_image = current_image[top:bottom, left:right]

                    # Save the cropped image
                    cv2.imwrite(f"{dir}/{count}.png", cropped_image)

                    count += 1

                if count > 100:
                    break


        else:
            pub.publish('WAITING FOR DETECT')

        rate.sleep()

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass

