# allegro_driver package

This package opens CAN bus to Allegro hand, subscribes to joint states topic and gives the robot hand desired joint angles while manually setting PD gains. The Plan_hand node is particular to the Allegro_moveit_config as it instantiates a move group node and sets target joint angles so that a path can be planned. 

## Setup ##
The following packages are meant to be used with ROS 2 Humble.
1. Make sure ROS packages are most recent and up-to-date
```
sudo apt update
sudo apt upgrade
```
2. Install dependencies:
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
3. Clone this repository to your workspace src.
4. In the CMAKEList.txt update the absolute path to libBHand.so in your grasping library.


