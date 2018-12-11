# OpenCV Changing Colorspaces demo - modified by George Allison for use in Furious Fowl project
# CSCI 3302 Robotics Fall 2018
# Requires opencv and python installed

import cv2, numpy as np

cap = cv2.VideoCapture(0)
count = 1
cLimit = 100 # Amount of iterations before printing coordinates, smaller number = faster refresh

print("Ctrl + c to exit program")
while(1):
    # Take each frame
    _, frame = cap.read()

    blueavg = None
    greenavg = None
    redavg = None

    # Convert BGR to HSV
    hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)

    # define range of blue color in HSV
    lower_blue = np.array([100,100,50])
    upper_blue = np.array([130,255,255])

    lower_green = np.array([20,100,50])
    upper_green = np.array([85,255,255])

    lower_red = np.array([160,100,50])
    upper_red = np.array([180,255,255])

    # Threshold the HSV image to get only blue colors
    bluemask = cv2.inRange(hsv, lower_blue, upper_blue)

    # Bitwise-AND mask and original image
    blueres = cv2.bitwise_and(frame,frame, mask= bluemask)

    # TODO: Return and X and Y coordinate for all unique color blobs
    bluepoints = cv2.findNonZero(bluemask)
    if bluepoints is not None:
        blueavg = np.mean(bluepoints, axis=0) # Averaged points to sole coordinate
        resImage = [133, 133]
        resScreen = [133, 133]

    #green
    greenmask = cv2.inRange(hsv, lower_green, upper_green)
    # Bitwise-AND mask and original image
    greenres = cv2.bitwise_and(frame,frame, mask= greenmask)
    # TODO: Return and X and Y coordinate for all unique color blobs
    greenpoints = cv2.findNonZero(greenmask)
    if greenpoints is not None:
        greenavg = np.mean(greenpoints, axis=0) # Averaged points to sole coordinate
        resImage = [133, 133]
        resScreen = [133, 133]

    #red
    redmask = cv2.inRange(hsv, lower_red, upper_red)
    # Bitwise-AND mask and original image
    redres = cv2.bitwise_and(frame,frame, mask= redmask)
    # TODO: Return and X and Y coordinate for all unique color blobs
    redpoints = cv2.findNonZero(redmask)
    if redpoints is not None:
        redavg = np.mean(redpoints, axis=0) # Averaged points to sole coordinate
        resImage = [133, 133]
        resScreen = [133, 133]

    if (count == cLimit):
        if (redavg is not None):
            rx_cm = redavg[0][0] / 4.51
            ry_cm = 96 - (redavg[0][1] / 4.6)
            print ("Red Coordinates: %.2f cm, %.2f cm" %(rx_cm, ry_cm))

        if (greenavg is not None):
            gx_cm = greenavg[0][0] / 4.51
            gy_cm = 96 - (greenavg[0][1] / 4.68)
            print ("Green Coordinates: %.2f cm, %.2f cm" %(gx_cm, gy_cm))


        if (blueavg is not None):
            bx_cm = blueavg[0][0] / 5
            by_cm = 96 - (blueavg[0][1] / 4.4)
            print ("Blue Coordinates: %.2f cm, %.2f cm" %(bx_cm, by_cm))

        count = 0
        
    cv2.imshow('frame',frame)
    #cv2.imshow('mask',mask)
    cv2.imshow('blueres',blueres)
    cv2.imshow('greenres',greenres)
    cv2.imshow('redres',redres)

    k = cv2.waitKey(5) & 0xFF
    if k == 27:
        break
    
    count = count + 1
    
cv2.destroyAllWindows()