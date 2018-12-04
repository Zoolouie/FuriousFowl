# OpenCV Changing Colorspaces demo - modified by George Allison for use in Furious Fowl project
# CSCI 3302 Robotics Fall 2018
# Requires opencv and python installed

import cv2, numpy as np

cap = cv2.VideoCapture(0)
count = 1
cLimit = 50 # Amount of iterations before printing coordinates, smaller number = faster refresh

print("Ctrl + c to exit program")
while(1):
    # Take each frame
    _, frame = cap.read()

    # Convert BGR to HSV
    hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)

    # define range of blue color in HSV
    lower_blue = np.array([100,100,100])
    upper_blue = np.array([130,255,255])

    # Threshold the HSV image to get only blue colors
    mask = cv2.inRange(hsv, lower_blue, upper_blue)

    # Bitwise-AND mask and original image
    res = cv2.bitwise_and(frame,frame, mask= mask)

    # TODO: Return and X and Y coordinate for all unique color blobs
    points = cv2.findNonZero(mask)
    avg = np.mean(points, axis=0) # Averaged points to sole coordinate
    resImage = [640, 480]
    resScreen = [1080, 400]
    pointInScreen = np.floor((resScreen[0] / resImage[0]) * avg[0]), np.floor((resScreen[1] / resImage[1]))

    #shitty way to have intermittant printing
    if (count == cLimit):
        print "Blue Coordinates: %s" % (pointInScreen,)
        count = 0

    #cv2.imshow('frame',frame)
    #cv2.imshow('mask',mask)
    cv2.imshow('res',res)
    k = cv2.waitKey(5) & 0xFF
    if k == 27:
        break
    
    count = count + 1
    
cv2.destroyAllWindows()