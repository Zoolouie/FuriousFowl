# OpenCV Changing Colorspaces demo - modified by George Allison for use in Furious Fowl project
# CSCI 3302 Robotics Fall 2018
# Requires opencv and python installed
import cv2, numpy as np
cap = cv2.VideoCapture(0)
cLimit = 100 # Amount of iterations before printing coordinates, smaller number = faster refresh

def main():
    coordlist = findCoords()
    print("Printing coordinate list...")
    
    #prints blue, green, red
    for x in coordlist:
        print x[0], x[1]

def findCoords():
    camBool = 0
    while camBool < 20:
        # Take each frame
        _, frame = cap.read()

        blueavg = None
        greenavg = None
        redavg = None

        # Convert BGR to HSV
        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)

        # define range of colors in HSV
        lower_blue = np.array([100,100,100])
        upper_blue = np.array([130,255,255])
        lower_green = np.array([80,100,100])
        upper_green = np.array([100,255,255])
        lower_red = np.array([0,100,100])
        upper_red = np.array([30,255,255])

        # Blue Mask
        bluemask = cv2.inRange(hsv, lower_blue, upper_blue)
        blueres = cv2.bitwise_and(frame,frame, mask= bluemask)
        bluepoints = cv2.findNonZero(bluemask)
        if bluepoints is not None:
            blueavg = np.mean(bluepoints, axis=0)

        # Green Mask
        greenmask = cv2.inRange(hsv, lower_green, upper_green)
        greenres = cv2.bitwise_and(frame,frame, mask= greenmask)
        greenpoints = cv2.findNonZero(greenmask)
        if greenpoints is not None:
            greenavg = np.mean(greenpoints, axis=0)
            resImage = [133, 133]
            resScreen = [133, 133]

        # Red  Mask
        redmask = cv2.inRange(hsv, lower_red, upper_red)
        redres = cv2.bitwise_and(frame,frame, mask= redmask)
        redpoints = cv2.findNonZero(redmask)
        if redpoints is not None:
            redavg = np.mean(redpoints, axis=0)

        # list of coordinates in centimeters
        coordlist = []

        # adding blue coordinates in centimeters
        if (blueavg is not None):
            bx_cm = int(blueavg[0][0] / 4.51)
            by_cm = int(96 - (blueavg[0][1] / 4.68))
            coordlist.append((bx_cm, by_cm))

        # adding green coordinates in centimeters
        if (greenavg is not None):
            gx_cm = int(greenavg[0][0] / 4.51)
            gy_cm = int(96 - (greenavg[0][1] / 4.68))
            coordlist.append((gx_cm, gy_cm))

        # adding red coordinates in centimeters
        if (redavg is not None):
            rx_cm = int(redavg[0][0] / 4.51)
            ry_cm = int(96 - (redavg[0][1] / 4.68))
            coordlist.append((rx_cm, ry_cm))

        #for showing masks and camera feeds:
        #cv2.imshow('frame',frame)
        #cv2.imshow('mask',mask)
        #cv2.imshow('blueres',blueres)
        #cv2.imshow('greenres',greenres)
        #cv2.imshow('redres',redres)

        camBool += 1
    # After While termination
    return coordlist
    #cv2.destroyAllWindows()

main()

