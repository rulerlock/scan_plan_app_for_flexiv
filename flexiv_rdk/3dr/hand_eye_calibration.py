#!/usr/bin/env python

""" Hand to Eye Calibration based on Flexiv RDK and open3d solver

Usage:
Move the  robot arm with calibration board. 

'c' to add a pose
'v' to validate
'q' to save and quit

"""

__copyright__ = "Copyright (C) 2016-2021 Flexiv Ltd. All Rights Reserved."
__author__ = "Flexiv"

import time
import argparse

# Import Flexiv RDK Python library
# fmt: off
import sys
sys.path.insert(0, "../lib_py")
import flexivrdk
# fmt: on

from SimpleHandEye.interfaces.cameras import RealSenseCamera
from SimpleHandEye.interfaces.apriltag import ApriltagTracker
from SimpleHandEye.solvers import OpenCVSolver
import cv2
import numpy as np

def showImage(color_frame, depth_frame, ir1_frame, ir2_frame):
    cv2.imshow('image', color_frame)
    cv2.waitKey(33)

def quaternion_to_rotation_matrix(q):
    qw, qx, qy, qz = q
    rotation_matrix = np.array([
        [1 - 2*qy**2 - 2*qz**2,   2*qx*qy - 2*qz*qw,       2*qx*qz + 2*qy*qw],
        [2*qx*qy + 2*qz*qw,       1 - 2*qx**2 - 2*qz**2,   2*qy*qz - 2*qx*qw],
        [2*qx*qz - 2*qy*qw,       2*qy*qz + 2*qx*qw,       1 - 2*qx**2 - 2*qy**2]
    ])
    return rotation_matrix

def homogeneous_transform(position, quaternion):
    x, y, z = position
    qw, qx, qy, qz = quaternion
    rotation_matrix = quaternion_to_rotation_matrix(quaternion)
    translation_vector = np.array([[x], [y], [z]])
    homogeneous_matrix = np.block([
        [rotation_matrix, translation_vector],
        [np.zeros((1, 3)), 1]
    ])
    return homogeneous_matrix


def main():
    # Program Setup
    # ==============================================================================================
    # Parse arguments
    argparser = argparse.ArgumentParser()
    argparser.add_argument("robot_ip", help="IP address of the robot server")
    argparser.add_argument("local_ip", help="IP address of this PC")
    args = argparser.parse_args()

    # Define alias
    log = flexivrdk.Log()
    mode = flexivrdk.Mode
    plan_info = flexivrdk.PlanInfo()

    camera = RealSenseCamera(callback_fn=showImage)

    intrinsics_params = camera.getIntrinsics()
    K = intrinsics_params['RGB']['K']
    D = intrinsics_params['RGB']['D']

    tag_pose_tracker = ApriltagTracker(tag_size=0.165, # put your tag size here
                            intrinsic_matrix=K,
                            distortion_coeffs=D)
    
    solver = OpenCVSolver(type='AX=YB')
    A_list = []
    B_list = []
    robot_states = flexivrdk.RobotStates()


    try:
        # Instantiate robot interface
        robot = flexivrdk.Robot(args.robot_ip, args.local_ip)

        # Clear fault on robot server if any
        if robot.isFault():
            log.warn("Fault occurred on robot server, trying to clear ...")
            # Try to clear the fault
            robot.clearFault()
            time.sleep(2)
            # Check again
            if robot.isFault():
                log.error("Fault cannot be cleared, exiting ...")
                return
            log.info("Fault on robot server is cleared")

        # Enable the robot, make sure the E-stop is released before enabling
        log.info("Enabling robot ...")
        robot.enable()

        # Wait for the robot to become operational
        while not robot.isOperational():
            time.sleep(1)

        

        # Switch to plan execution mode
        robot.setMode(mode.NRT_PLAN_EXECUTION)
        
        robot.executePlan(22, True)
        log.info("Robot is now operational")
        log.info("Press 'c' to add a pose, 'v' to validate, 'q' to save and quit")

        while True:

            user_input = input()

            if user_input == "c":
                robot.getRobotStates(robot_states)
                cam_T_tag = tag_pose_tracker.getPose(camera.color_frame, tag_id=1)
                if cam_T_tag is None:
                    print("tag not found")
                    continue
                B_list.append(cam_T_tag)
                pose = robot_states.tcpPose
                A_list.append(homogeneous_transform(pose[:3], pose[3:7]))
                print("added pose")

            elif user_input == "v":
                X, Y = solver.solve(A_list, B_list)
                print("hand_T_tag=")
                print(X)
                print("base_T_camera=")
                print(Y)

                base_T_camera = np.array(Y)
                hand_T_tag = np.array(X)
                robot.getRobotStates(robot_states)
                cam_T_tag = tag_pose_tracker.getPose(camera.color_frame, tag_id=1)
                pose = robot_states.tcpPose
                base_T_hand = np.array(homogeneous_transform(pose[:3], pose[3:7]))
                print("Check validation result, which should be an identity matrix:")
                print(base_T_hand @ hand_T_tag @ np.linalg.inv(cam_T_tag) @ np.linalg.inv(base_T_camera))

            elif user_input == "q":
                with open('./3dr/base_T_camera.txt', 'w') as file:
                    np.savetxt(file, base_T_cam)
                print("Pose saved to 3dr/base_T_camera.txt. The Cam intrisics is",K)
                break

            else:
                continue

        X, Y = solver.solve(A_list, B_list)
        print("hand_T_tag=")
        print(X)
        print("base_T_camera=")
        print(Y)

        # base_T_camera = np.array(Y)
        # hand_T_tag = np.array(X)
        # robot.getRobotStates(robot_states)
        # cam_T_tag = tag_pose_tracker.getPose(camera.color_frame, tag_id=1)
        # pose = robot_states.tcpPose
        # base_T_hand = np.array(homogeneous_transform(pose[:3], pose[3:7]))
        # print("validation:")
        # print(base_T_hand @ hand_T_tag @ np.linalg.inv(cam_T_tag) @ np.linalg.inv(base_T_camera))

    except Exception as e:
        # Print exception error message
        log.error(str(e))
    exit()


if __name__ == "__main__":
    main()
