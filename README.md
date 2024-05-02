# Control_twizzy

This package is created for UTAC Challenge by ENSTA team 2023-2024 and is responsible for the communication between the [StreetDrone](https://streetdrone.com/) vehicles and ROS based self-driving software stacks. It's based on the repository of StreetDrone (https://github.com/streetdrone-home/SD-VehicleInterface). 
The focus of this repo is to be used by later groups and to have a more generilised step-by-step with different ubuntu versions, and additional steps in our experience for the functioning of the vehicle connection.

### Disclaimer:
We strongly suggest that you adhere to the following guideline in conjuction with the documentation you received alongside your vehicle
* A trained safety driver must always be present in the vehicle, monitoring the behavior of the vehicle and ready to take control at any time.
* This software is for research purposes only. Access to and use of this software is at your own risk and responsibility. No warranty expressed or implied.
* You are responsible for complying with the local laws and regulations.

#### In this release:
* A tunable PID and FeedForward Linear Velocity Control loop has been implemened with a mature calibration at speeds of up to 20mph. 
* Support has been extended to the ENV200, the latest vehicle in the StreetDrone fleet
* An intuitive yaw to steering map has been included
* Support for Localization, CAN and IMU speed source selection
* Support for OXTS and PEAK GPS-IMU
* Simulation mode for [SD-TwizyModel](https://github.com/streetdrone-home/SD-TwizyModel) in Gazebo
* Minor bug fixes

## Requirements

##### - Ubuntu 16.04 LTS / Ubuntu 18.04 LTS / Ubuntu 20.04 LTS

##### - ROS 
#### - Kinetic [ros-kinetic-desktop-full](http://wiki.ros.org/kinetic/Installation/Ubuntu) (Ubuntu 16.04 LTS)
#### - Melodic [ros-kinetic-desktop-full](http://wiki.ros.org/kinetic/Installation/Ubuntu) (Ubuntu 18.04 LTS)
#### - Noetic [ros-kinetic-desktop-full](http://wiki.ros.org/kinetic/Installation/Ubuntu) (Ubuntu 20.04 LTS)

##### - Catkin Command Line Tools [catkin_tools](https://catkin-tools.readthedocs.io/en/latest/installing.html)

## STEPS

# UBUNTU - DUAL BOOT


## Building

1. Go to the `/src` directory of your catkin workspace and clone this repository or copy its contents.   
If you haven't previously created a catkin workspace, please visit [wiki.ros.org/Tutorials/create_a_workspace](http://wiki.ros.org/catkin/Tutorials/create_a_workspace) 

2. Install the ROS packages: [socketcan_interface](http://wiki.ros.org/socketcan_interface) and [can_msgs](http://wiki.ros.org/can_msgs)
```
sudo apt-get install ros-kinetic-socketcan-interface ros-kinetic-can-msgs
```

3. Build the package
```
# from the root of your workspace build only this package
catkin build sd_vehicle_interface
```

If you previously built your workspace with `catkin_make`, do `catkin_make --only-pkg-with-deps sd_vehicle_interface`.    

Launch
------
Before launching, ensure that the CAN interface has been initialised.  
For PEAK CAN USB interfaces, the steps to initialise CAN as can0, are:
```
sudo modprobe peak_usb
sudo ip link set can0 up type can bitrate 500000
```
If not connected to the car, virtual CAN Bus can be used. The steps to initialise virtual CAN as vcan, are:
```
sudo modprobe vcan
sudo ip link add dev can0 type vcan
```
After CAN is initialised, go to the root of your workspace and launch the vehicle interface using the following command:
```
source devel/setup.bash
roslaunch sd_vehicle_interface sd_vehicle_interface.launch sd_vehicle:=env200 sd_gps_imu:=oxts
# adjust the launch parameters to your vehicle setup, as described below
```

Parameters
------

| arg                | values                                    | default           | description                                |
|--------------------|-------------------------------------------|-------------------|--------------------------------------------|
| sd_vehicle         | {env200,twizy}                            | env200            | The vehicle under control                  |
| sd_gps_imu         | {oxts, peak, none}                        | oxts              | The GPS/IMU used                           |
| sd_speed_source    | {vehicle_can_speed, imu_speed, ndt_speed} | vehicle_can_speed | Input vehicle speed                        |
| sd_simulation_mode | {true, false}                             | false             | Use on the car or on the Gazebo simulation |

Control 
------
To implement control of the vehicle and integrate autonomous driving algorithms, we only need to use Python scripts to receive and send vehicle control information. The step 
is as follow:
1. Place the .py file in the folder \Home Folder\catkin_ws\src\SD-VehicleInterface\vehicle_interface\scripts.

Make a modification in the file \Home Folder\catkin_ws\src\SD-VehicleInterface\vehicle_interface\CMakeLists.txt:
