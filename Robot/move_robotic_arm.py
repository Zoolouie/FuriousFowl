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
    gripper.get_position()
    gripper.set_position(0)

def slingshot_math():
 	return

def cost_function():
	return

def image_processing():
	return

def prepare_slingshot():
    move_to_initial()
    open_gripper()
    time.sleep(5) 
    close_gripper()

def move_to_initial():
    position = [-0.13, 0.43, 0.01]
    quaternion = [0.0, 1.0, 0.0, 0.0]
    moveRoboticArm(position, quaternion, 0.05, 0.05)

def move_to_point(position, orientation):
    position = [-0.22, 0.43, -0.02]
    quaternion = [0.0, 1.0, 0.0, 0.0]
    moveRoboticArm(position, quaternion, 0.05, 0.05)
    time.sleep(5)
    open_gripper()    

def main():
    rospy.init_node('go_to_cartesian_pose_py')   
    prepare_slingshot()
    # move_to_point(1,2)
    

main()