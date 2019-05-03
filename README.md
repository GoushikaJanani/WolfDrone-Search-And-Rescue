# Search-And-Rescue (SAR) Drone

The goal of this project is to develop and demonstrate an UAV capable of recognizing a "body" or person lying on the ground and then land and deliver a small "rescue" package next to that person.

### Prerequisites

1. Python 2.7 
2. OpenCV 3.4.2
3. Darknet YOLOv3
4. ROS Kinetic
5. v4l2capture
6. MAVROS

### Installing

1. Install all prerequisites required in Jetson TX2
2. Go to terminal. mkdir catkin_ws
3. cd catkin_ws
4. copy src folder from github into this folder
5. catkin_make
5. source devel/setup.bash

## Running the tests

Once the prerequisites are installed on Jetson TX2,run roslaunch mavros px4.launch in another terminal.

If you want to run the whole search and rescue mission run roslaunch wolfdrone search_and_rescue.launch.

If you want to test a take off and land run roslaunch wolfdrone takeoff_land.launch.
