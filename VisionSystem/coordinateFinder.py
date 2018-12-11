# OpenCV Changing Colorspaces demo - modified by George Allison for use in Furious Fowl project
# CSCI 3302 Robotics Fall 2018
# Requires opencv and python
#
# Second attempt at isolation of vision system function for use in final program

import cv2, numpy as np

def main():

    # The visionSystem function returns a list of three tuples,
    # X and Y coordinates in centimeters for red, green, blue,
    # in that order. The main function of this program prints
    # the content of the list:

    coordinates = visionSystem()
    print("Printing coordinate list...")
    
    #prints blue, green, red
    for x in coordinates:
        print x[0], x[1]

def visionSystem():
    cap = cv2.VideoCapture(1) # video capture is used instead of snapshot to more accurately average slightly moving objects
    count = 1 #iteration count
    cLimit = 100 # Number of iterations before printing; smaller number = faster printing/refresh
    run = True # Used to terminate program

    while(run):
        _, frame = cap.read() # read camera frame, recommended cLimit > 10 to account for camera latency 
    
        # initializing variables that will be used to storing averages
        blueavg = None
        greenavg = None
        redavg = None
    
        # Convert BGR to HSV, info on HSV: https://alloyui.com/examples/color-picker/hsv
        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
    
        # define upper and lower bounds of color ranges in HSV
        lower_blue = np.array([100,100,50])
        upper_blue = np.array([130,255,255])
    
        lower_green = np.array([20,100,50])
        upper_green = np.array([85,255,255])
    
        lower_red = np.array([160,100,50])
        upper_red = np.array([180,255,255])
    
        # Threshold the HSV image to get only blue colors
        bluemask = cv2.inRange(hsv, lower_blue, upper_blue)
    
        # Bitwise-AND mask with original image to distinguish color
        blueres = cv2.bitwise_and(frame,frame, mask= bluemask)
    
        bluepoints = cv2.findNonZero(bluemask)
        if bluepoints is not None: # Color mask may be NONE when cLimit is too small or camera is over/underexposed
            blueavg = np.mean(bluepoints, axis=0) # Average of points in mask
            resImage = [133, 133]
            resScreen = [133, 133]
    
        # same thing for green:
        greenmask = cv2.inRange(hsv, lower_green, upper_green)
        greenres = cv2.bitwise_and(frame,frame, mask= greenmask)
        greenpoints = cv2.findNonZero(greenmask)
        if greenpoints is not None:
            greenavg = np.mean(greenpoints, axis=0)
            resImage = [133, 133]
            resScreen = [133, 133]
    
        # same thing for red:
        redmask = cv2.inRange(hsv, lower_red, upper_red)
        redres = cv2.bitwise_and(frame,frame, mask= redmask)
        redpoints = cv2.findNonZero(redmask)
        if redpoints is not None:
            redavg = np.mean(redpoints, axis=0) 
            resImage = [133, 133]
            resScreen = [133, 133]
    
    
        # My amatuer way of having intermittant printing
        # cLimit determines rate of printing
        # To mimic a snapshot, we quit the program and return coordinates after one execution
        if (count == cLimit):
            # list of coordinate tuples
            coordList = []
    
            # X and Y pixel to cm conversions for red
            rx_cm = int(redavg[0][0] / 4.51)
            ry_cm = int(96 - (redavg[0][1] / 4.68))
            if (redavg is not None):
                coordList.append((rx_cm, ry_cm))
            
            # X and Y pixel to cm conversions for green        
            gx_cm = int(greenavg[0][0] / 4.51)
            gy_cm = int(96 - (greenavg[0][1] / 4.68))
            if (greenavg is not None):
                coordList.append((gx_cm, gy_cm))
    
            # X and Y pixel to cm conversions for blue        
            bx_cm = int(blueavg[0][0] / 4.51)
            by_cm = int(96 - (blueavg[0][1] / 4.68))
            if (blueavg is not None):
                coordList.append((bx_cm, by_cm))

            return coordList
    
            run = False # Exit loop
    
    
        # Uncomment to see camera feed and live color masks:
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

main()
