# Allegro_hand
Winter project, manipulating the ALlegro Hand by mimicing hand movements

## Install necessary packages.

- sudo apt-get install cmake gcc g++ libpopt-dev

1. Download, build, and install PCAN-USB driver for Linux: libpcan. Note: v7.9 and v7.13 are tested.
tar -xzvf peak-linux-driver-x.x.tar.gz
cd peak-linux-driver-x.x
make NET=NO
sudo make install
sudo modprobe pcan

2. Download, build, and install PCAN-Basic API for Linux: libpcanbasic. Note: v2.0.3 is tested.
tar -xzvf PCAN_Basic_Linux-x.x.x.tar.gz
cd PCAN_Basic_Linux-x.x.x/pcanbasic
make
sudo make install

3. Download, build, and install Grasping Library for Linux, "libBHand": Grasping_Library_for_Linux

4. Go to the CMAKELists.txt of allegro_driver and change the path in line 15 of the grasping library

5. Connect PCAN-USB and Allegro Hand (make sure to power off Allegro Hand)

6. Power on Allegro Hand.

7.  Run the allegro_driver node

