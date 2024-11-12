# Flexiv RDK

![CMake Badge](https://github.com/flexivrobotics/flexiv_rdk/actions/workflows/cmake.yml/badge.svg)
[![License](https://img.shields.io/badge/License-Apache%202.0-blue.svg)](https://www.apache.org/licenses/LICENSE-2.0.html)

This project mainly includes two components: 
1. [Python RDK Based] Grip object, scanning, point cloud processing and trimming pose generation
2. [ROS 2 Based] Trajectory planning and execution



## References

[Flexiv RDK Home Page](https://rdk.flexiv.com/) is the main reference. It contains important information including environment setup and API documentation.

Flexiv RDK (Robotic Development Kit), a key component of the Flexiv Robotic Software Platform, is a powerful development toolkit that enables the users to create complex and customized robotic applications using APIs that provide both low-level real-time (RT) and high-level non-real-time (NRT) access to Flexiv robots.

## Compatibility Overview

| **Supported OS**           | **Supported processor** | **Supported language** | **Required compiler kit** |
| -------------------------- | ----------------------- | ---------------------- | ------------------------- |
| Linux (Ubuntu       22.04) | x86_64, arm64           | C++, Python            | build-essential           |


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

![Project Workflow]('/flexiv_rdk/doc')

The work flow is packed into a main panel. To start, run:

        python3 scan_ui.py

Key in robot arm IP and local IP. Follow the sequece of buttons to:
1) Initialise and grip the object.
2) Move robot arm to scan. And save the point clouds & poses.
3) View registered scan result. 
4) Upload CAD model's point cloud and match the orientation.
5) Find and cluster difference pieces, calculate the trimming pose.

#### ROS 2 Trimming Execution

1. Install Microsoft Visual Studio with version 2015 or above (MSVC 14.0+). Choose the "Desktop development with C++" package during installation.
2. Download ``cmake-3.x.x-windows-x86_64.msi`` from [CMake download page](https://cmake.org/download/) and install the msi file. The minimum required version is 3.16.3. **Add CMake to system PATH** when prompted, so that ``cmake`` and ``cmake-gui`` command can be used from Command Prompt or a bash emulator.
3. Install a bash emulator. Git Bash that comes with Git (for Windows) installation is recommended.
4. Within the bash emulator, the rest steps are the same as [Compile and install for Linux](#compile-and-install-for-linux), beginning from step 2.




#### Compile and install for Linux

1. Modify the trimming pose at:

        cd /flexiv_ros2_ws/src/flexiv_ros2/flexiv_bringup2/src/main1.cpp

2. Assuming the setup detailed in the Flexiv ROS 2 bridge is done, compile the program: 

        cd /flexiv_ros2_ws
        source /opt/ros/humble/setup.bash
        colcon build --symlink-install

   Note: ``sudo`` is not required unless prompted by the program saying "root privilege is required".

2. Initialise the MoveIt 2 enviornment:

        ros2 launch flexiv_bringup rizon_moveit.launch.py robot_ip:=192.168.3.100 local_ip:=192.168.3.163
        colcon build --packages-select flexiv_bringup2

   If you don't run on a real robot, run simulation instead:

        ros2 launch flexiv_bringup2 rizon_moveit.launch.py robot_ip:=dont-care local_ip:=dont-care use_fake_hardware:=true

3. Run the planning script:

        ros2 launch flexiv_bringup2 rizon_main.launch.py robot_ip:=dont-care local_ip:=dont-care use_fake_hardware:=true



ros2 launch flexiv_bringup2 rizon_main.launch.py robot_ip:=192.168.3.100 local_ip:=192.168.3.163 use_fake_hardware:=false

#### Compile and install for macOS

1. In a Terminal, use ``xcode-select`` command to invoke the installation of Xcode Command Line Tools, then follow the prompted window to finish the installation.
2. Download ``cmake-3.x.x-macos-universal.dmg`` from [CMake download page](https://cmake.org/download/) and install the dmg file. The minimum required version is 3.16.3.
3. When done, start CMake from Launchpad and navigate to Tools -> How to Install For Command Line Use. Then follow the instruction "Or, to install symlinks to '/usr/local/bin', run:" to install ``cmake`` and ``cmake-gui`` command for use in Terminal.
4. The rest steps are the same as [Compile and install for Linux](#compile-and-install-for-linux), beginning from step 2.
