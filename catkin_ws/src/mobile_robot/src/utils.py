import cv2
import numpy as np
# import matplotlib.pyplot as plt
from math import sqrt
from skimage.feature import blob_dog, blob_log, blob_doh
import imutils
import argparse
import os
import math

from classification import training, getLabel
from classification import SVM

# SIGNS = ["ERROR",
#         "STOP",
#         "TURN LEFT",
#         "TURN RIGHT",
#         "DO NOT TURN LEFT",
#         "DO NOT TURN RIGHT",
#         "ONE WAY",
#         "SPEED LIMIT",
#         "OTHER"]

SIGNS = ["STOP",
        "TURN LEFT",
        "TURN RIGHT",
        "ONE WAY"]

# Clean all previous file
def clean_images():
	file_list = os.listdir('./')
	for file_name in file_list:
		if '.png' in file_name:
			os.remove(file_name)


### Preprocess image
def constrastLimit(image):
    img_hist_equalized = cv2.cvtColor(image, cv2.COLOR_BGR2YCrCb)
    channels = cv2.split(img_hist_equalized)
    channels = list(channels)  # Explicitly convert to list if necessary
    channels[0] = cv2.equalizeHist(channels[0])
    img_hist_equalized = cv2.merge(channels)
    img_hist_equalized = cv2.cvtColor(img_hist_equalized, cv2.COLOR_YCrCb2BGR)
    return img_hist_equalized


def LaplacianOfGaussian(image):
    LoG_image = cv2.GaussianBlur(image, (3,3), 0)           # paramter 
    gray = cv2.cvtColor( LoG_image, cv2.COLOR_BGR2GRAY)
    LoG_image = cv2.Laplacian( gray, cv2.CV_8U,3,3,2)       # parameter
    LoG_image = cv2.convertScaleAbs(LoG_image)
    return LoG_image
    
def binarization(image):
    thresh = cv2.threshold(image,32,255,cv2.THRESH_BINARY)[1]
    #thresh = cv2.adaptiveThreshold(image,255,cv2.ADAPTIVE_THRESH_GAUSSIAN_C, cv2.THRESH_BINARY,11,2)
    return thresh

def preprocess_image(image):
    image = constrastLimit(image)
    image = LaplacianOfGaussian(image)
    image = binarization(image)
    return image

# Find Signs
def removeSmallComponents(image, threshold):
    #find all your connected components (white blobs in your image)
    nb_components, output, stats, centroids = cv2.connectedComponentsWithStats(image, connectivity=8)
    sizes = stats[1:, -1]; nb_components = nb_components - 1

    img2 = np.zeros((output.shape),dtype = np.uint8)
    #for every component in the image, you keep it only if it's above threshold
    for i in range(0, nb_components):
        if sizes[i] >= threshold:
            img2[output == i + 1] = 255
    return img2

def findContour(image):
    #find contours in the thresholded image
    cnts = cv2.findContours(image, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_NONE    )
    
    # cnts = cnts[0] if imutils.is_cv2() else cnts[1]
    cnts = cnts[0]
    # print(cnts)
    return cnts

def contourIsSign(perimeter, centroid, threshold):
    #  perimeter, centroid, threshold
    # # Compute signature of contour
    result=[]
    for p in perimeter:
        p = p[0]
        distance = sqrt((p[0] - centroid[0])**2 + (p[1] - centroid[1])**2)
        result.append(distance)
    max_value = max(result)
    signature = [float(dist) / max_value for dist in result ]
    # Check signature of contour.
    temp = sum((1 - s) for s in signature)
    temp = temp / len(signature)
    if temp < threshold: # is  the sign
        return True, max_value + 2
    else:                 # is not the sign
        return False, max_value + 2

#crop sign 
def cropContour(image, center, max_distance):
    width = image.shape[1]
    height = image.shape[0]
    top = max([int(center[0] - max_distance), 0])
    bottom = min([int(center[0] + max_distance + 1), height-1])
    left = max([int(center[1] - max_distance), 0])
    right = min([int(center[1] + max_distance+1), width-1])
    print(left, right, top, bottom)
    return image[left:right, top:bottom]

def cropSign(image, coordinate):
    width = image.shape[1]
    height = image.shape[0]
    top = max([int(coordinate[0][1]), 0])
    bottom = min([int(coordinate[1][1]), height-1])
    left = max([int(coordinate[0][0]), 0])
    right = min([int(coordinate[1][0]), width-1])
    #print(top,left,bottom,right)
    return image[top:bottom,left:right]


def findLargestSign(image, contours, threshold, distance_theshold):
    max_distance = 0
    coordinate = None
    sign = None
    for c in contours:
        print("Contour shape:", c.shape)
        print("Contour type:", type(c))
        c = c.reshape(-1, 1, 2)
        c = np.array(c, dtype=np.int32)

        M = cv2.moments(c)
        if M["m00"] == 0:
            continue
        cX = int(M["m10"] / M["m00"])
        cY = int(M["m01"] / M["m00"])
        is_sign, distance = contourIsSign(c, [cX, cY], 1-threshold)
        if is_sign and distance > max_distance and distance > distance_theshold:
            max_distance = distance
            coordinate = np.reshape(c, [-1,2])
            left, top = np.amin(coordinate, axis=0)
            right, bottom = np.amax(coordinate, axis = 0)
            coordinate = [(left-2,top-2),(right+3,bottom+1)]
            sign = cropSign(image,coordinate)
    return sign, coordinate


def findSigns(image, contours, threshold, distance_theshold):
    signs = []
    coordinates = []
    for c in contours:
        M = cv2.moments(c)
        if M["m00"] == 0:
            continue
        cX = int(M["m10"] / M["m00"])
        cY = int(M["m01"] / M["m00"])
        is_sign, max_distance = contourIsSign(c, [cX, cY], 1-threshold)
        if is_sign and max_distance > distance_theshold:
            sign = cropContour(image, [cX, cY], max_distance)
            signs.append(sign)
            coordinate = np.reshape(c, [-1,2])
            top, left = np.amin(coordinate, axis=0)
            right, bottom = np.amax(coordinate, axis = 0)
            coordinates.append([(top-2,left-2),(right+1,bottom+1)])
    return signs, coordinates

# def localization(image, min_size_components, similitary_contour_with_circle, model, count, current_sign_type):
#     original_image = image.copy()
#     binary_image = preprocess_image(image)

#     binary_image = removeSmallComponents(binary_image, min_size_components)

#     binary_image = cv2.bitwise_and(binary_image,binary_image, mask=remove_other_color(image))

#     #binary_image = remove_line(binary_image)

#     # cv2.imshow('BINARY IMAGE', binary_image)
#     contours = findContour(binary_image)
#     #signs, coordinates = findSigns(image, contours, similitary_contour_with_circle, 15)
#     sign, coordinate = findLargestSign(original_image, contours, similitary_contour_with_circle, 15)
    
#     text = ""
#     sign_type = -1
#     i = 0

#     if sign is not None:
#         sign_type = getLabel(model, sign)
#         sign_type = sign_type if sign_type <= 8 else 8
#         text = SIGNS[sign_type]
#         # cv2.imwrite(str(count)+'_'+text+'.png', sign)

#     # if sign_type > 0 and sign_type != current_sign_type:        
#     #     cv2.rectangle(original_image, coordinate[0],coordinate[1], (0, 255, 0), 1)
#     #     font = cv2.FONT_HERSHEY_PLAIN
#     #     cv2.putText(original_image,text,(coordinate[0][0], coordinate[0][1] -15), font, 1,(0,0,255),2,cv2.LINE_4)
#     return coordinate, original_image, sign_type, text

def remove_line(img):
    gray = img.copy()
    edges = cv2.Canny(gray,50,150,apertureSize = 3)
    minLineLength = 5
    maxLineGap = 3
    lines = cv2.HoughLinesP(edges,1,np.pi/180,15,minLineLength,maxLineGap)
    mask = np.ones(img.shape[:2], dtype="uint8") * 255
    if lines is not None:
        for line in lines:
            for x1,y1,x2,y2 in line:
                cv2.line(mask,(x1,y1),(x2,y2),(0,0,0),2)
    return cv2.bitwise_and(img, img, mask=mask)

def remove_other_color(img):
    frame = cv2.GaussianBlur(img, (3,3), 0) 
    hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
    # define range of blue color in HSV
    lower_blue = np.array([100,128,0])
    upper_blue = np.array([215,255,255])
    # Threshold the HSV image to get only blue colors
    mask_blue = cv2.inRange(hsv, lower_blue, upper_blue)

    lower_white = np.array([0,0,128], dtype=np.uint8)
    upper_white = np.array([255,255,255], dtype=np.uint8)
    # Threshold the HSV image to get only blue colors
    mask_white = cv2.inRange(hsv, lower_white, upper_white)

    lower_black = np.array([0,0,0], dtype=np.uint8)
    upper_black = np.array([170,150,50], dtype=np.uint8)

    mask_black = cv2.inRange(hsv, lower_black, upper_black)

    mask_1 = cv2.bitwise_or(mask_blue, mask_white)
    mask = cv2.bitwise_or(mask_1, mask_black)
    # Bitwise-AND mask and original image
    #res = cv2.bitwise_and(frame,frame, mask= mask)
    return mask


def draw_picture(image, coordinate, sign_type, text):
    # Check if there's a detected coordinate and sign type
    if coordinate is not None: #and sign_type > 0:
        # Draw rectangle and text on the image
        cv2.rectangle(image, coordinate[0], coordinate[1], (0, 255, 0), 1)
        font = cv2.FONT_HERSHEY_PLAIN
        cv2.putText(image, text, (coordinate[0][0], coordinate[0][1] - 15), font, 1, (0, 0, 255), 2)

    return image

# Replace the following with your actual localization function implementation
def localization(image, min_size_components, similitary_contour_with_circle, model):
    original_image = image.copy()
    binary_image = preprocess_image(image)

    binary_image = removeSmallComponents(binary_image, min_size_components)

    binary_image = cv2.bitwise_and(binary_image,binary_image, mask=remove_other_color(image))

    #binary_image = remove_line(binary_image)

    # cv2.imshow('BINARY IMAGE', binary_image)
    contours = findContour(binary_image)
    #signs, coordinates = findSigns(image, contours, similitary_contour_with_circle, 15)
    sign, coordinate = findLargestSign(original_image, contours, similitary_contour_with_circle, 15)
    
    text = ""
    sign_type = -1
    i = 0

    if sign is not None:
        sign_type = getLabel(model, sign)
        sign_type = sign_type if sign_type <= 8 else 8
        text = SIGNS[sign_type]
       
        cv2.rectangle(original_image, coordinate[0],coordinate[1], (0, 255, 0), 1)
        font = cv2.FONT_HERSHEY_PLAIN
        cv2.putText(original_image,text,(coordinate[0][0], coordinate[0][1] -15), font, 1,(0,0,255),2,cv2.LINE_4)
    
    return coordinate, original_image, sign_type, text

def localization_and_draw(image, min_size_components, similitary_contour_with_circle, model):
    # Assume localization function returns the needed parameters
    coordinate, image, sign_type, text = localization(image, min_size_components, similitary_contour_with_circle, model)
    
    print("coordinate: ", coordinate)
    print("sign_type: ", sign_type)
    print("text: ", text)

    # Call draw_picture with the obtained parameters
    image = draw_picture(image, coordinate, sign_type, text)

    return image

# Example usage
if __name__ == '__main__':
    # Load model and other necessary setups
    # model = SVM()
    # model.load("data_svm.dat")
    # model = training()
    model = cv2.ml.SVM_load("data_svm.dat")
    
    # Example image processing
    image = cv2.imread('example.png')
    image = cv2.resize(image, (640,480))
    processed_image = localization_and_draw(image, 300, 0.65, model)
    # cv2.imshow('Processed Image', processed_image)
    # cv2.waitKey(0)
    # cv2.destroyAllWindows()
    cv2.imwrite('result.png', processed_image)