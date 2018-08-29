# camera_driver
ROS driver for Camera Devices

## Prerequisite
 * Ubuntu 16.04
 * ROS Kinetic
 
## How to Fetch
` $ cd ~/catkin_ws/src`

`$ git clone https://github.intel.com/drones/camera_driver.git`

## How to Build
`$ cd ~/catkin_ws`

`$ catkin build camera_driver`

## How to Run
`$ rosrun camera_driver camera_driver_node`

## How to Test
1. Run following command and whether camera ROS topics are listed

   `$ rostopic list`

2. Run following command and check whether all GTests are passed

   `$ catkin run_tests camera_driver`
   
   `$ catkin_test_results camera_driver`
