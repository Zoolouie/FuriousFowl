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

    lower_green = np.array([80,100,100])
    upper_green = np.array([100,255,255])

    lower_red = np.array([0,100,100])
    upper_red = np.array([30,255,255])

    # Threshold the HSV image to get only blue colors
    bluemask = cv2.inRange(hsv, lower_blue, upper_blue)

    # Bitwise-AND mask and original image
    blueres = cv2.bitwise_and(frame,frame, mask= bluemask)

    # TODO: Return and X and Y coordinate for all unique color blobs
    bluepoints = cv2.findNonZero(bluemask)
    blueavg = np.mean(bluepoints, axis=0) # Averaged points to sole coordinate
    resImage = [640, 480]
    resScreen = [1080, 400]
    bluepointsinframe = np.floor((resScreen[0] / resImage[0]) * blueavg[0]), np.floor((resScreen[1] / resImage[1]))
    bluex = bluepointsinframe[0][0]
    bluex = float(bluex)
    bluey = bluepointsinframe[0][1]
    bluey = float(bluey)


   # greenmask = cv2.inRange(hsv, lower_green, upper_green)
   # greenres = cv2.bitwise_and(frame,frame, mask= greenmask)
#
   # greenpoints = cv2.findNonZero(greenmask)
   # greenavg = np.mean(greenpoints, axis=0) # Averaged points to sole coordinate
   # resImage = [640, 480]
   # resScreen = [1080, 400]
   # greenpointsinframe = np.floor((resScreen[0] / resImage[0]) * greenavg[0]), (np.floor((resScreen[1] / resImage[1])))
#
#
   # redmask = cv2.inRange(hsv, lower_red, upper_red)
   # redres = cv2.bitwise_and(frame,frame, mask= redmask)
#
   # redpoints = cv2.findNonZero(redmask)
   # redavg = np.mean(redpoints, axis=0) # Averaged points to sole coordinate
   # resImage = [640, 480]
   # resScreen = [1080, 400]
   # redpointsinframe = np.floor((resScreen[0] / resImage[0]) * redavg[0]), np.floor((resScreen[1] / resImage[1]))

    
    

    '''
    greenx = greenpointsinframe[0][0]
    greenx = float(greenx)
    greeny = greenpointsinframe[0][1]
    greeny = float(greeny)
    redx = redpointsinframe[0][0]
    redx = float(redx)
    redy = redpointsinframe[0][1]
    redy = float(redy)
    '''


    #shitty way to have intermittant printing
    if (count == cLimit):
        print "blue coordinates: %s , %s" % (bluex, bluey)
       # print "green coordinates: %s , %s" % (greenx, greeny)
       # print "red coordinates %s , %s" % (redx, redy)
        count = 0

    #cv2.imshow('frame',frame)
    #cv2.imshow('mask',mask)
    #cv2.imshow('blueres',blueres)
    #cv2.imshow('greenres',greenres)
    #cv2.imshow('redres',redres)

    k = cv2.waitKey(5) & 0xFF
    if k == 27:
        break
    
    count = count + 1
    
cv2.destroyAllWindows()