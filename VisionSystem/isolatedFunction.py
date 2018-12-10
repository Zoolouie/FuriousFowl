# OpenCV Changing Colorspaces demo - modified by George Allison for use in Furious Fowl project
# CSCI 3302 Robotics Fall 2018
# Requires opencv and python installed
import cv2, numpy as np
cap = cv2.VideoCapture(1)
cLimit = 100 # Amount of iterations before printing coordinates, smaller number = faster refresh

def main():
    coordlist = findCoords()
    print("Printing coordinate list...")
    
    #prints blue, green, yellow
    for x in coordlist:
        print x[0], x[1]

def findCoords():
    camBool = 0
    # camera latency calls for use of while loop instead of just capturing one frame
    while camBool < 20:
        # Take each frame
        _, frame = cap.read()

        blueavg = None
        greenavg = None
        yellowavg = None

        # Convert BGR to HSV
        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)

        # define range of colors in HSV
        lower_blue = np.array([100,100,100])
        upper_blue = np.array([130,255,255])
        lower_green = np.array([80,100,100])
        upper_green = np.array([120,255,255])
        lower_yellow = np.array([10,100,100])
        upper_yellow = np.array([35,255,255])

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

        # yellow  Mask
        yellowmask = cv2.inRange(hsv, lower_yellow, upper_yellow)
        yellowres = cv2.bitwise_and(frame,frame, mask= yellowmask)
        yellowpoints = cv2.findNonZero(yellowmask)
        if yellowpoints is not None:
            yellowavg = np.mean(yellowpoints, axis=0)

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

        # adding yellow coordinates in centimeters
        if (yellowavg is not None):
            rx_cm = int(yellowavg[0][0] / 4.51)
            ry_cm = int(96 - (yellowavg[0][1] / 4.68))
            coordlist.append((rx_cm, ry_cm))

        camBool += 1
    # After While termination
    return coordlist
    #cv2.destroyAllWindows()

main()

