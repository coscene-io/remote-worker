1 Installing ROS2 Applications for REMOTE WORKER on Ubuntu 22.04 (ROS2 Humble)

1.1 Downloading the Project and Installing Dependencies

Update software sources:
```
  sudo apt update
  Upgrade the system:
  sudo apt upgrade
  Install git:
  sudo apt install git
```
Then, clone the project from GitHub:
```
git clone https://github.com/coscene-io/remote-worker
```
Install dependencies:
```
  sudo apt install software-properties-common
  sudo add-apt-repository universe
  sudo apt update && sudo apt install curl -y
  sudo curl -sSL   https://raw.githubusercontent.com/ros/rosdistro/master/ros.key   -o /usr/share/keyrings/ros-archive-keyring.gpg
  echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg]   http://packages.ros.org/ros2/ubuntu   $(. /etc/os-release && echo $UBUNTU_CODENAME) main" | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null
  sudo apt update
  sudo apt upgrade
  sudo apt install ros-humble-desktop
  sudo apt install ros-dev-tools
  sudo apt install net-tools
  sudo apt install ros-humble-moveit-*
  sudo apt install ros-humble-foxglove-bridge
  sudo apt autoremove ros-humble-moveit-servo-*
```
Add ROS2 to the source Source the setup script:
```
  echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc
  source ~/.bashrc
```
Install Python3 libraries:
```
  sudo apt install python3-pip
  mkdir ~/remote_worker
  cd ~/remote_worker
  python3 -m pip install -r requirements.txt
``` 
Initial compilation:
```
  cd ~/remote_worker
  sudo chmod +x build_first.sh
  . build_first.sh
```  
Contents of build_first.sh (automatically run by the script; no manual execution required). This step may take a while to complete:
```
  cd ~/remote_worker
  colcon build
  echo "source ~/remote_worker/install/setup.bash" >> ~/.bashrc
  source ~/.bashrc 
```  
1.2 Subsequent Compilations and Usage

Every time you make changes to a package's code, you need to recompile it before using it.

Regular compilation:
```
  cd ~/remote_worker
  . build_common.sh
```  
Contents of build_common.sh (automatically run by the script; no manual execution required):
```
  cd ~/remote_worker
  colcon build
  source install/setup.bash 
```
---
2 remote_worker Package Overview

remote_worker is a workspace containing multiple ROS2 packages, each serving a specific purpose in the operation and control of robotic arms. Below is an overview of each package and its main functionalities:

1.roarm_driver Driver for Real Robot:

  Responsible for interfacing with and controlling the physical robotic arm hardware.
  
2.roarm_description Robotic Arm Model:

  Contains the URDF (Unified Robot Description Format) files and other model descriptions necessary for simulating and visualizing the robotic arm.
  
3.roarm_moveit Kinematic Configuration:

  Provides configurations for MoveIt, a motion planning framework, including setup files and parameters required for the kinematic control of the robotic arm.
  
4.roarm_moveit_ikfast_plugins IKFast Kinematics Solver:

Implements the IKFast kinematics solver, which is used for efficient and fast inverse kinematics calculations.

5.ik_solver Inverse Kinematics Service Node

  This package provides a node that receives target poses for the robotic arm, computes the corresponding joint angles using IKFast or a custom inverse kinematics solver, and publishes the results for use by the robot driver or higher-level control modules.
  
6.sirius_reader Sensor Data Processing

  This package handles incoming data from external sensors such as vision, localization, or force sensors, performs coordinate transformations to align the sensor data with the robot’s workspace, and publishes the resulting target poses for motion planning and inverse kinematics.
7.roarm_bringup Integrated Launch System

  This package serves as the central launch entry point for the entire project, providing launch files that start up the robot driver, model descriptions, MoveIt configurations, sensor processing nodes, and inverse kinematics solver. It streamlines system initialization and enables seamless switching between simulation and real hardware.

---
3 Controlling the Physical Robotic Arm with Sensor

3.1 Connect the Devices and Identify the Serial Port
Before connecting the robotic arm and sensor via USB, check the current serial devices on your Ubuntu system:
```
  ls /dev/tty*
```
Then, connect the robotic arm and sensor. Ensure your computer (Windows) can detect the USB devices. Remember to always connect the Sirius sensor before the connection of robotic arm.
Check the serial devices again:
```
  ls /dev/tty*
```
You should now see new devices named /dev/ttyUSB0 and /dev/ttyUSB1 at the end of the list. If not, disconnect and reconnect the devices.

3.2 Change the Serial Port Device

If the detected serial port device are /dev/ttyUSB0 and /dev/ttyUSB1, you can skip this section.

If the serial port device is not /dev/ttyUSB0 and /dev/ttyUSB1, you need to update the serial port device name in the Python scripts 

"~/remote_worker/src/sirius_reader/sirius_reader/sirius_reader.py" by changing line 54:
```
serial_port = "/dev/ttyUSB0"
```
"~/remote_worker/src/roarm_driver/roarm_driver/roarm_driver.py" by changing line 15:
```
serial_port = "/dev/ttyUSB1"
```
to your actual serial port device names.
Then, recompile the ROS2 packages in the terminal:
```
  cd ~/roarm_ws_em0/
  colcon build
  source install/setup.bash
```
3.3 Running the project Nodes

According to the ROS2 official documentation, it is not recommended to run ROS2 nodes in the same terminal where you compile the packages. Open a new terminal window using ```Ctrl + Alt + T```.
Grant serial port permissions and run the ROS2 robotic arm driver node:

Grant read and write permissions to the serial device using the following command (replace /dev/ttyUSB0 and /dev/ttyUSB1 with your actual device path):
```
sudo chmod 666 /dev/ttyUSB0
sudo chmod 666 /dev/ttyUSB1
Launch the nodes:
source install/setup.bash
ros2 launch roarm_bringup bringup.launch.py
```
