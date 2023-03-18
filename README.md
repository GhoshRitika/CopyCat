# CopyCat: Movement Mirroring with Allegro Hand
Author: Ritika Ghosh

[GraspingWithAllegro](https://user-images.githubusercontent.com/60728026/226075105-37d79943-52b6-4db4-8c63-c11b21a8843d.mp4)

## **Description**
The CopyCat package enables the 4 fingered Wonick robotics’ Allegro hand to mirror the finger movement of an actual hand in ROS 2. This package has 2 modes, one where the robot hand is capable of mimicking gross finger movement while the other mode allows for the recognition of 5 types of hand gestures which are useful for carrying out finer grasping tasks. In order to perform these tasks the package utilizes and RGB camera to observe the human hand movements tracked using mediapipe’s machine learning framework to either employ an algorithm to calculate and retarget the joint states directly or use a pre-trained hand gesture recognition package to obtain already defined corresponding grasping hand configuration. These 16 joint state angles are fed to the Moveit! planner to plan and execute valid hand movements.

## **Hardware Requirements**
1. [Wonik Allegro Robotic Hand Version 4.0](http://wiki.wonikrobotics.com/AllegroHandWiki/index.php/Allegro_Hand_v4.0)
2. [PCAN-USB Adapter](https://www.peak-system.com/PCAN-USB.199.0.html?&L=1)
*Note: Allegro code is written for PCAN devices*

## **Software Dependencies**
1. [Moveit](https://moveit.picknik.ai/humble/index.html)
2. [MediaPipe](https://google.github.io/mediapipe/solutions/hands.html)
3. [Bhand Library](http://wiki.wonikrobotics.com/AllegroHandWiki/index.php/Grasping_Library_for_Linux)
4. [PCAN usb driver](http://www.peak-system.com/fileadmin/media/linux/index.htm#download)
5. [PCAN api](https://www.peak-system.com/Software-APIs.305.0.html?&L=1)

## **Setup Guidelines**
The following packages are meant to be used with ROS 2 Humble.
1. Make sure ROS packages are most recent and up-to-date
```
sudo apt update
sudo apt upgrade
```
2. Install Moveit!: `sudo apt install ros-humble-moveit`
3. Create a new ROS2 workspace and enter it
```
mkdir -p copycatws/src 
cd copycatws
```
4. Install dependencies:
```
sudo apt-get install cmake gcc g++ libpopt-dev
pip install media pipe
sudo apt-get install python3-opencv
```
- Download, build, and install PCAN-USB driver for Linux: [libpcan](http://www.peak-system.com/fileadmin/media/linux/index.htm#download)
    ```
    tar -xzvf peak-linux-driver-x.x.tar.gz
    cd peak-linux-driver-x.x
    make NET=NO
    sudo make install
    sudo modprobe pcan
    ```
- Download, build, and install PCAN-Basic API for Linux: [libpcanbasic](https://www.peak-system.com/Software-APIs.305.0.html?&L=1)
    ```
    tar -xzvf PCAN_Basic_Linux-x.x.x.tar.gz
    cd PCAN_Basic_Linux-x.x.x/pcanbasic
    make
    sudo make install
    ```
- Download, build, and install Grasping Library for Linux, "libBHand": [Grasping_Library_for_Linux](http://wiki.wonikrobotics.com/AllegroHandWiki/index.php/Grasping_Library_for_Linux)
5. Download [this package](https://github.com/GhoshRitika/Allegro_hand) into the src directory: `git clone git@github.com:GhoshRitika/Allegro_hand.git`
6. Download my [gesture recognition](https://github.com/GhoshRitika/go1-gesture-command) fork in the same src directory: `git clone git@github.com:GhoshRitika/go1-gesture-command.git`

## **Contents**
Refer to README of individual packages for more detailed instructions and information.
1. [Allegro_hand](https://github.com/GhoshRitika/Allegro_hand)
    - [allegro_driver](https://github.com/GhoshRitika/Allegro_hand/tree/main/allegro_driver) : ROS2 C++
    - [Allegro_moveit_config](https://github.com/GhoshRitika/Allegro_hand/tree/main/Allegro_moveit_config)
    - [allegro_lib](https://github.com/GhoshRitika/Allegro_hand/tree/main/allegro_lib): C++
    - [finger_tracking](https://github.com/GhoshRitika/Allegro_hand/tree/main/finger_tracking): ROS 2 python
2. [go1-gesture-command](https://github.com/GhoshRitika/go1-gesture-command)
    - [go1_cmd](https://github.com/GhoshRitika/go1-gesture-command/tree/main/go1_cmd): ROS 2 python
    - [ros2_hgr](https://github.com/GhoshRitika/go1-gesture-command/tree/main/ros2_hgr): ROS2 python

## **User Guide**
1. Connect PCAN-USB and Allegro Hand (make sure to power off Allegro Hand)
2. Power on Allegro Hand using the toggle switch on the side.
3. To use the CopyCat package in the first mode, i.e to visually operate the hand movement:
`ros2 launch allegro_driver launch_all.launch.xml teleop:=true`
4. To use the gesture recognition mode:
`ros2 launch allegro_driver launch_all.launch.xml teleop:=false`
`ros2 launch ros2_hgr hgr.launch.xm`
Begin moving your right hand in front of your webcam!

## **Package Structure**
### Visual Feedback: 
**finger_tracking:**
Uses mediapipe’s hand recognition framework to calculate each of the joint angles of the hand configuration as seen by the RGB camera and publishes it for the motion controller.

**go1-gesture-recognition:**
Uses a fork of the mediapipe gesture recognition repository to correctly recognize a gesture made by the right hand and assign an id number to it. Depending on the id the hgr_com node publishes the joint angles for corresponding grasping configuration in the robot hand.
### Motion Controller:
**allegro_driver:**
It has 2 nodes, the plan_hand node subscribes to the joint_angles topic and sends it to the move group node to plan a trajectory and then execute it. The allegro_driver node subscribes to the joint_states topic and sends the resulting angles to the robot hand.

**Allegro_moveit_config:**
Configured the Allegro Hand robot with MoveIt! With the help of MoveIt setup assistant such that its controller can plan and execute valid trajectories for the nearest solution while avoiding self collision and exceeding finger joint limits given goal joint angles. 
