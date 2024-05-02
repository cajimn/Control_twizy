# Control_twizy

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

## UBUNTU - DUAL BOOT

To make the communication between the car and the bus can we must use ROS, for this, at the beginning none of the team members had Ubuntu installed, so we tried to use a virtual machine or download it to windows but then the code generated problems. For this reason the only viable solution that was found was to make a DUAL BOOT on the computer, in this way we would keep our windows operating system and we would also have the ubuntu we wanted.
For this, keep in mind that all the documentation is based on ROS 1, so the only Ubuntu versions that handle this version are the 3 already mentioned.

Performing the dual boot is really easy and we can follow the instructions in the following video: [Dual boot](https://www.youtube.com/watch?v=tEh1RfmbTBY) or find another of your preference. 


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
1. Place the .py file in the folder `\Home Folder\catkin_ws\src\SD-VehicleInterface\vehicle_interface\scripts`.
2. Make a modification in the file `\Home Folder\catkin_ws\src\SD-VehicleInterface\vehicle_interface\CMakeLists.txt`:
* At the very bottom, add:
```
catkin_install_python(PROGRAMS scripts/<name of the file.py>
    DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
```
3. Re-run `catkin_make`.
4. It's possible to modify a Python program directly, without needing to run `catkin_make` each time, once the above steps are done. Simply save the file as you go and launch it.
5. To launch the Python program, open another terminal and run the following commands:
```
cd catkin_ws
source devel/setup.bash
cd src/SD-VehicleInterface/vehicle_interface/scripts
python <file.py>
```

We have created a demo version of script Python in our repository. After checking the control message in the folder sd_msgs, we found that the control information should 
include two parameters which are `torque` and `steer` and represent respectively speed and angle of wheel. So the steps of control are:
1. Create a publisher object of type `SDControl`.
```
pub = rospy.Publisher('sd_control', SDControl, queue_size=1)
```
2. Change the value of control parameters
```
# Create an instance of the SDControl message
control_msg = SDControl()
control_msg.header = Header()
control_msg.header.stamp = rospy.Time.now()
control_msg.torque = 50  # Adjust as needed for desired speed
control_msg.steer = 0  # Go straight
```
3. Publish the control message
```
while not rospy.is_shutdown():
 pub.publish(control_msg)
# Wait for the next time to publish
 rate.sleep()
```
