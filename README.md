# Flexiv Scan Plan App

![CMake Badge](https://github.com/flexivrobotics/flexiv_rdk/actions/workflows/cmake.yml/badge.svg)
[![License](https://img.shields.io/badge/License-Apache%202.0-blue.svg)](https://www.apache.org/licenses/LICENSE-2.0.html)

This project mainly includes two components: 
1. [Python RDK Based] Grip object, scanning, point cloud processing and trimming pose generation
2. [ROS 2 Based] Trajectory planning and execution

![Project Workflow]('/flexiv_rdk/doc/Project_workflow.png')

## References

[Flexiv RDK Home Page](https://rdk.flexiv.com/) is the main reference. It contains important information including environment setup and API documentation.

Flexiv RDK (Robotic Development Kit), a key component of the Flexiv Robotic Software Platform, is a powerful development toolkit that enables the users to create complex and customized robotic applications using APIs that provide both low-level real-time (RT) and high-level non-real-time (NRT) access to Flexiv robots.

## Compatibility Overview

| **Supported OS**           | **Supported processor** | **Supported language** | **Required compiler kit** |
| -------------------------- | ----------------------- | ---------------------- | ------------------------- |
| Linux (Ubuntu       22.04) | x86_64, arm64           | C++, Python            | build-essential           |

**RDK Version** 
v0.10.0

**ROS Version** 
ROS 2 Humble Hawksbill


## Quick Start

**NOTE:** the full documentation is in [Flexiv RDK Manual](https://rdk.flexiv.com/manual/).

Python 3.8 and 3.10 are supported by RDK, see Flexiv RDK Manual for more details. A brief instruction is provided below.

Check Python version is 3.8.x or 3.10.x:

        python3 --version

### Hand eye calibration

Assume the system setup detailed in Flexiv RDK Manual is done, to start:

        cd flexiv_rdk/3dr/
        python3 hand_eye_calibration.py

Uppon successful calibration, the trasformation matrix will be saved to /3dr/base_T_camera.txt

### Scan, match CAD and calculate trimming pose

The work flow is packed into a main panel. To start, run:

        python3 scan_ui.py

Key in robot arm IP and local IP. Follow the sequece of buttons to:
1) Initialise and grip the object.
2) Move robot arm to scan. And save the point clouds & poses.
3) View registered scan result. 
4) Upload CAD model's point cloud and match the orientation.
5) Find and cluster difference pieces, calculate the trimming pose.

#### ROS 2 Trimming Execution

Use the calculating result of trimming parts' poses as the input of OMPL planning of Rizon 4 robot arm in ROS 2 MoveIt environment.

#### Compile and install for Linux

To begin, the environment setup of Flexiv ROS 2 bridge should be done. See [Flexiv ROS 2 Package](https://rdk.flexiv.com/manual/ros2_packages.html)

1. Modify the trimming poses in script main1.cpp:

        /flexiv_ros2_ws/src/flexiv_ros2/flexiv_bringup2/src/main1.cpp

2. Assuming the setup detailed in the Flexiv ROS 2 bridge is done, compile the program: 

        cd /flexiv_ros2_ws
        source /opt/ros/humble/setup.bash
        colcon build --symlink-install

   Note: ``sudo`` is not required unless prompted by the program saying "root privilege is required".

3. Initialise the MoveIt 2 enviornment:

        ros2 launch flexiv_bringup rizon_moveit.launch.py robot_ip:=192.168.3.100 local_ip:=192.168.3.163
        colcon build --packages-select flexiv_bringup2

   If you don't run on a real robot, run simulation instead:

        ros2 launch flexiv_bringup2 rizon_moveit.launch.py robot_ip:=dont-care local_ip:=dont-care use_fake_hardware:=true

4. Run the planning script:

        ros2 launch flexiv_bringup2 rizon_main.launch.py robot_ip:=dont-care local_ip:=dont-care use_fake_hardware:=true

   If you don't run on a real robot, run simulation instead:
   
        ros2 launch flexiv_bringup2 rizon_main.launch.py robot_ip:=192.168.3.100 local_ip:=192.168.3.163 use_fake_hardware:=false

5. If the path is sovled, there will be log "Path computed successfully. Waiting for confirming". Check the path on RViZ. Change the parameter to execute on robot:

        ros2 param set main1 continue_execution true    

#### File Tree

flexiv_rdk
├── 3dr
│   ├── base_T_camera.txt
│   ├── datareader.py
│   ├── gripper_init.py
│   ├── hand_eye_calibration.py
│   ├── hgtm.py
│   ├── lib_py
│   ├── pc_process_1.py
│   ├── pc_scan.py
│   ├── pointcloud_scan.py
│   ├── realsense
│   ├── save_txt.py
│   └── scan_ui.py
├── cmake
├── CMakeLists.txt
├── doc
├── example
├── example_py
│   ├── basics1_display_robot_states.py
│   ├── basics2_clear_fault.py
│   ├── basics3_primitive_execution.py
│   ├── basics4_plan_execution.py
│   ├── basics5_zero_force_torque_sensors.py
│   ├── basics6_gripper_control.py
│   ├── basics7_auto_recovery.py
│   ├── intermediate1_non_realtime_joint_position_control.py
│   ├── intermediate2_non_realtime_cartesian_pure_motion_control.py
│   ├── intermediate3_non_realtime_cartesian_motion_force_control.py
│   ├── intermediate4_teach_by_demonstration.py
│   └── utility.py
├── file_rdk.txt
├── include
├── lib
├── lib_py
├── librealsense-2.55.1
├── LICENSE
├── README.md
├── requirements.txt
├── resources
├── scan_output
│   ├── poses.txt
│   ├── scans
│   └── tcp.txt
├── test
└── thirdparty

flexiv_ros2_ws
├── build
├── file_tree.txt
├── install
├── log
└── src
    └── flexiv_ros2
        ├── build
        ├── flexiv_bringup
        ├── flexiv_bringup2	# Package written for this project
        │   ├── CMakeLists.txt
        │   ├── config
        │   ├── launch
        │   │   ├── rizon.launch.py
        │   │   ├── rizon_main.launch.py	# Launch file for trimming trajectory planning
        │   │   ├── rizon_moveit_constrain.launch.py
        │   │   ├── rizon_moveit.launch.py	# Main launch file for RViZ env
        │   │   ├── sine_sweep_impedance.launch.py
        │   │   ├── sine_sweep_position.launch.py
        │   │   └── test_joint_trajectory_controller.launch.py
        │   ├── package.xml
        │   ├── README.md
        │   └── src
        │       └── main1.cpp
        ├── flexiv_controllers
        ├── flexiv_description
        ├── flexiv_hardware
        ├── flexiv_moveit_config
        │   ├── CMakeLists.txt
        │   ├── config
        │   │   ├── joint_limits.yaml
        │   │   ├── kinematics.yaml
        │   │   ├── moveit_controllers.yaml
        │   │   ├── ompl_planning.yaml		# Edited OMPL planner configs
        │   │   └── rizon_moveit_servo_config.yaml
        │   ├── package.xml
        │   ├── rviz
        │   └── srdf
        ├── flexiv_msgs
        ├── flexiv_test_nodes
        ├── install
        ├── LICENSE
        ├── log
        └── README.md