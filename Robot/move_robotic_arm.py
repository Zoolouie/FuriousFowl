#! /usr/bin/env python
import rospy
import intera_interface
from intera_motion_interface import (
    MotionTrajectory,
    MotionWaypoint,
    MotionWaypointOptions
)
from intera_motion_msgs.msg import TrajectoryOptions
from geometry_msgs.msg import PoseStamped
import PyKDL
from tf_conversions import posemath
from intera_interface import Limb
import time
import cv2 
import numpy as np
import math

def moveRoboticArm(position, orientation, linear_speed, linear_accel):
    """
    Move the robot arm to the specified configuration given a positionX, positionY, positionZ, quaternion array and max linear speed.

    """
    try:
        limb = Limb()

        traj_options = TrajectoryOptions()
        traj_options.interpolation_type = TrajectoryOptions.CARTESIAN
        traj = MotionTrajectory(trajectory_options = traj_options, limb = limb)

        wpt_opts = MotionWaypointOptions(max_linear_speed=linear_speed, max_linear_accel=linear_accel)
        waypoint = MotionWaypoint(options = wpt_opts.to_msg(), limb = limb)
        joint_names = limb.joint_names()
        endpoint_state = limb.tip_state('right_hand')
        pose = endpoint_state.pose
        if position is not None and len(position) == 3:
            pose.position.x = position[0]
            pose.position.y = position[1]
            pose.position.z = position[2]
        if orientation is not None and len(orientation) == 4:
            pose.orientation.x = orientation[0]
            pose.orientation.y = orientation[1]
            pose.orientation.z = orientation[2]
            pose.orientation.w = orientation[3]
        poseStamped = PoseStamped()
        poseStamped.pose = pose
        joint_angles = limb.joint_ordered_angles()
        waypoint.set_cartesian_pose(poseStamped, 'right_hand', joint_angles)
        rospy.loginfo('Sending waypoint: \n%s', waypoint.to_string())
        traj.append_waypoint(waypoint.to_msg())
        result = traj.send_trajectory()

        if result is None:
            rospy.logerr('Trajectory FAILED to send')
            return
        if result.result:
            rospy.loginfo('Motion controller successfully finished the trajectory!')
        else:
            rospy.logerr('Motion controller failed to complete the trajectory with error %s',
                         result.errorId)
    except rospy.ROSInterruptException:
        print("Something went wrong")
        rospy.logerr('Keyboard interrupt detected from the user. Exiting before trajectory completion.')


def open_gripper():
    robot_state = intera_interface.RobotEnable()
    gripper = intera_interface.Gripper('right_gripper')
    gripper.get_position()
    gripper.set_position(100)

def close_gripper():
    gripper = intera_interface.Gripper('right_gripper')
    gripper.set_cmd_velocity(gripper.MAX_VELOCITY)
    print(gripper.MAX_VELOCITY)
    gripper.get_position()
    gripper.set_position(0.0025)


def cost_function():
	return

def prepare_slingshot():
    move_to_initial()
    open_gripper()
    time.sleep(3) 
    close_gripper()

def move_to_initial():
    #open_gripper()
    position = [-0.11, 0.437, -0.01]
    quaternion = [0.0, 1.0, 0.0, 0.0]
    moveRoboticArm(position, quaternion, 0.05, 0.05)

def move_to_point(position, position2):
    position = [position, 0.437, position2]
    quaternion = [0.0, 1.0, 0.0, 0.0]
    moveRoboticArm(position, quaternion, 0.05, 0.05)
    time.sleep(1)
    open_gripper()    

def calculateTargetXAndY(Cans, Target):
    NotHittable = Cans
    NotHittable.remove(Target)
    gravity = 9.8;
    test=False
    velocities = []
    max_vel = 55 #10*5.3
    #print((5.3 * math.sin((20)) * 0.32) - ((1/2) * (gravity * (0.32 * 0.32))))
    #print(math.sin((20)),0.5)
    for veliter in range(max_vel,0,-1):
        velocities.append(float(veliter)/10)
    #print(velocities)
    for theta in range(90):
        #for vel in velocties:
        #This can do a continuous range decreasing from our max velocity
         for vel in velocities:
            max_time = (2 * vel * math.sin(math.radians(theta))) / gravity
            #print(max_time,vel,theta)
            max_time = int(math.ceil(100*max_time))
            #print(math.sin(math.radians(theta)),theta)
            #print(max_time)

            time_step = []
            for timeIter in range(max_time):
                #test=float(timeIter)/100
                #print(test)
                time_step.append(float(timeIter)/100)
            #print(time_step)
            for t in time_step:
                #test = True
                x = vel * math.cos(math.radians(theta)) * t
                y = (vel * math.sin(math.radians(theta)) * t) - ((0.5) * (gravity * (t ** 2)))
                #print(x,y,vel,theta)
                #Convert x y into cm
                #Turn it into whole numbers and subtract distance between camera and frame
                x=round(x*100)-87
                y=round(y*100)
                #print(x,y)
                #print(x,y)
                if (doesXYIntercept(x,y,NotHittable)):
                    break
                elif (doesXYCollide(x,y,Target)):
                    #print(vel, theta)
                    #print(x,y)
                    xf,yf=calculateBotPosition(vel, theta)
                    #print(final_position)
                    return xf,yf
    print('This position cant be hit')
    return False
    

def calculateBotPosition(vel, theta):
    #These two need to be find, mass of ball is 0.07 grams and the change of the spring with a 983 grams was 4 cm
    springk = 0.983/0.04
    mass = 0.007
    changeX = (((vel ** 2) * mass) / springk)**0.5
    x = changeX * math.cos(math.radians(theta))
    y = changeX * math.sin(math.radians(theta))
    return x, y

def doesXYIntercept(x,y,Obsticles):
    for i in Obsticles:
        #print(x,y,i)
        if x <= i[0]+3 and i[0]-3 <= x:
	    	if y <= i[1]+10 and i[1] <= y:
	        	return True
    return False

def doesXYCollide(x,y,Target):
    #print(x,y,Target)
    if x <= Target[0]+3 and Target[0]-3 <= x:
    	if y <= Target[1]+10 and Target[1] <= y:
        	return True
   	return False


def visionSystem():
    cap = cv2.VideoCapture(0) # video capture is used instead of snapshot to more accurately average slightly moving objects
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
            #rx_cm = int(redavg[0][0] / 4.51)
            #ry_cm = int(96 - (redavg[0][1] / 4.68))
            #if (redavg is not None):
                #coordList.append((rx_cm, ry_cm))
            
            # X and Y pixel to cm conversions for green        
            #gx_cm = int(greenavg[0][0] / 4.51)
            #gy_cm = int(96 - (greenavg[0][1] / 4.68))
            #if (greenavg is not None):
                #coordList.append((gx_cm, gy_cm))
    
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
  
def calculate_lowest_point(coordinates):
	min_point = -1
	min_index = 999
	for i in coordinates:
		if i[0] < min_index:
			min_point = i[0]
			min_index = i
	return min_index



	

def main():
    time.sleep(5)
    target_coordinates = visionSystem()
    #print(target_coordinates)
    #Cost Function Assuming no cans change ever, TODO replace with cost function
    target = calculate_lowest_point(target_coordinates)
    robot_coords1,robot_coords2 = calculateTargetXAndY(target_coordinates, target)
    #robot_coords1=robot_coords1+0.2
    #robot_coords2=robot_coords2+0.2

    rospy.init_node('go_to_cartesian_pose_py')   
    prepare_slingshot()

    move_to_point(-0.11-robot_coords1,-0.01-robot_coords2-0.01)
    

main()
