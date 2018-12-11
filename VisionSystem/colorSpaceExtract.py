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
    lower_blue = np.array([100,100,100])
    upper_blue = np.array([130,255,255])

    lower_green = np.array([80,100,100])
    upper_green = np.array([120,255,255])

    lower_red = np.array([0,80,100])
    upper_red = np.array([7,100,100])

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
        #resImage = [600, 600]
        #resScreen = [600, 600]
        #bluepointsinframe = np.floor((resScreen[0] / resImage[0]) * blueavg[0]), np.floor((resScreen[1] / resImage[1]))
        #bluex = bluepointsinframe[0][0]
        #bluex = float(bluex)
        #bluey = bluepointsinframe[0][1]
        #bluey = float(bluey)

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
        #resImage = [600, 600]
        #resScreen = [600, 600]

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
        bx_cm = blueavg[0][0] / 4.51
        by_cm = 96 - (blueavg[0][1] / 4.68)
        if (blueavg is not None):
            print ("Blue Coordinates: %.2f cm, %.2f cm" %(bx_cm, by_cm))

        gx_cm = greenavg[0][0] / 4.51
        gy_cm = 96 - (greenavg[0][1] / 4.68)
        if (greenavg is not None):
            print ("Green Coordinates: %.2f cm, %.2f cm" %(gx_cm, gy_cm))

        rx_cm = redavg[0][0] / 4.51
        ry_cm = 96 - (redavg[0][1] / 4.68)
        if (redavg is not None):
            print ("red Coordinates: %.2f cm, %.2f cm" %(rx_cm, ry_cm))

        #printing pixels
        #if (blueavg is not None):
        #    print ("blueavg %s" %(blueavg[0],))
        #if (greenavg is not None):
        #    print ("greenavg %s" %(greenavg[0],))
        #if (redavg is not None):
        #    print ("redavg %s" %(redavg[0],))
       # print "blue coordinates: %s , %s" % (bluex, bluey)
       # print "green coordinates: %s , %s" % (greenx, greeny)
       # print "red coordinates %s , %s" % (redx, redy)
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