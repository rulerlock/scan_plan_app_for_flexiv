#!/usr/bin/env python

"""basics4_plan_execution.py

This tutorial executes a plan selected by the user from a list of available plans. A plan is a
pre-written script to execute a series of robot primitives with pre-defined transition conditions
between 2 adjacent primitives. Users can use Flexiv Elements to compose their own plan and assign
to the robot, which will appear in the plan list.
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


def print_description():
    """
    Print tutorial description.

    """
    print(
        "This tutorial executes a plan selected by the user from a list of available "
        "plans. A plan is a pre-written script to execute a series of robot primitives "
        "with pre-defined transition conditions between 2 adjacent primitives. Users can "
        "use Flexiv Elements to compose their own plan and assign to the robot, which "
        "will appear in the plan list."
    )
    print()


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

    # Print description
    log.info("Tutorial description:")
    print_description()

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
        # RDK Initialization
        # ==========================================================================================
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

        log.info("Robot is now operational")

        # Execute Plans
        # ==========================================================================================
        # Switch to plan execution mode
        robot.setMode(mode.NRT_PLAN_EXECUTION)
        log.info("Executing plan")
        robot.executePlan(22, True)

        while True:
            # Monitor fault on robot server
            # if robot.isFault():
            #     raise Exception("Fault occurred on robot server, exiting ...")

            # Get user input
            # log.info("Choose an action:")
            # print("[1] Show available plans")
            # print("[2] Execute a plan by index")
            # print("[3] Execute a plan by name")

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
                # print("hand_T_tag=")
                # print(X)
                # print("base_T_camera=")
                # print(Y)

                base_T_camera = np.array(Y)
                hand_T_tag = np.array(X)
                robot.getRobotStates(robot_states)
                cam_T_tag = tag_pose_tracker.getPose(camera.color_frame, tag_id=1)
                pose = robot_states.tcpPose
                base_T_hand = np.array(homogeneous_transform(pose[:3], pose[3:7]))
                print("validation:")
                print(base_T_hand @ hand_T_tag @ np.linalg.inv(cam_T_tag) @ np.linalg.inv(base_T_camera))

            elif user_input == "q":
                print("Cam intrisics",K)
                break



            # Get and show plan list
            # if user_input == 1:
            #     plan_list = robot.getPlanNameList()
            #     for i in range(len(plan_list)):
            #         print("[" + str(i) + "]", plan_list[i])
            #     print("")

            # Execute plan by index
            # elif user_input == 2:
            #     index = int(input("Enter plan index to execute:\n"))
            #     # Allow the plan to continue its execution even if the RDK program is closed or
            #     # the connection is lost
            #     robot.executePlan(index, True)

            #     # Print plan info while the current plan is running
            #     while robot.isBusy():
            #         robot.getPlanInfo(plan_info)
            #         log.info(" ")
            #         print("assignedPlanName: ", plan_info.assignedPlanName)
            #         print("ptName: ", plan_info.ptName)
            #         print("nodeName: ", plan_info.nodeName)
            #         print("nodePath: ", plan_info.nodePath)
            #         print("nodePathTimePeriod: ", plan_info.nodePathTimePeriod)
            #         print("nodePathNumber: ", plan_info.nodePathNumber)
            #         print("velocityScale: ", plan_info.velocityScale)
            #         print("")
            #         time.sleep(1)

            # # Execute plan by name
            # elif user_input == 3:
            #     name = str(input("Enter plan name to execute:\n"))
            #     # Allow the plan to continue its execution even if the RDK program is closed or
            #     # the connection is lost
            #     robot.executePlan(name, True)

            #     # Print plan info while the current plan is running
            #     while robot.isBusy():
            #         robot.getPlanInfo(plan_info)
            #         log.info(" ")
            #         print("assignedPlanName: ", plan_info.assignedPlanName)
            #         print("ptName: ", plan_info.ptName)
            #         print("nodeName: ", plan_info.nodeName)
            #         print("nodePath: ", plan_info.nodePath)
            #         print("nodePathTimePeriod: ", plan_info.nodePathTimePeriod)
            #         print("nodePathNumber: ", plan_info.nodePathNumber)
            #         print("velocityScale: ", plan_info.velocityScale)
            #         print("")
            #         time.sleep(1)

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
