#!/usr/bin/env python3

import rospy
from std_msgs.msg import Bool, String
import cv2
from utils.classification import SVM, getLabel
from utils.utils import *
import os

# Initialize global variables
start_detection = False
sign_counts = {}

def load_model(model_path):
    svm_model = SVM()
    svm_model.load(model_path)
    return svm_model

def callback_start_detection(data):
    global start_detection
    start_detection = data.data

def detect_signs(model, display_results):
    for i in range(1, 11):
        image = cv2.imread(os.path.join("images", f"{i}.jpg"))
        if image is not None:
            coordinate, sign_type, text = localization(image, 300, 0.65, model)
            
            # Optionally display the results based on the ROS argument
            if display_results:
                processed_image = draw_picture(image, coordinate, sign_type, text)
                cv2.imwrite(f'detection_{i}.png', processed_image)
                cv2.imshow(f'detection_{i}.png', processed_image)

            sign_counts[text] = sign_counts.get(text, 0) + 1

    # Find the maximum count
    max_count = max(sign_counts.values(), default=0)

    # Find the first key with this maximum count
    for key, value in sign_counts.items():
        if value == max_count:
            return key

def main():
    global model, pub, sub

    # Load the trained SVM model
    # model = load_model("data_svm.dat")
    model = training()

    # Initialize ROS node
    rospy.init_node('traffic_sign_detector', anonymous=True)

    # ROS Publisher and Subscriber
    pub = rospy.Publisher('traffic_sign', String, queue_size=10)
    sub = rospy.Subscriber('start_detection', Bool, callback_start_detection)

    # Set the rate of the loop
    rate = rospy.Rate(10)  # 10hz

    rospy.sleep(1)

    display_results = rospy.get_param("~display", False)

    while not rospy.is_shutdown():
        if start_detection:
            most_common_sign = detect_signs(model, display_results)
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
