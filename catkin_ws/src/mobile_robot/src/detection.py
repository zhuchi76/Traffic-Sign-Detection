#!/usr/bin/env python3

import rospy
from std_msgs.msg import Bool, String
import cv2
from classification import SVM, getLabel
from utils import *

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
        image_path = f'{i}.png'
        image = cv2.imread(image_path)
        if image is not None:
            coordinate, sign_type, text = localization(image, min_size_components, similitary_contour_with_circle, model)
            
            # Optionally display the results based on the ROS argument
            if display_results:
                image = draw_picture(image, coordinate, sign_type, text)
                cv2.imshow(f'Detection {i}', processed_image)
                cv2.imwrite(f'detection_{i}.png', processed_image)

            sign_counts[text] = sign_counts.get(text, 0) + 1

    return max(sign_counts, key=sign_counts.get)
    

def publish_result(sign_type):
    pub.publish(str(sign_type))
    rospy.loginfo(f'Published Most Common Traffic Sign: {sign_type}')

def main():
    global model, pub, sub

    # Load the trained SVM model
    model = load_model("data_svm.dat")

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
        # The main loop where you can add more functionality
        if start_detection:
            most_common_sign = detect_signs(model, display_results)
            publish_result(most_common_sign)
        rate.sleep()

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass



