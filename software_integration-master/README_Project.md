Our system is based on provided software_integration source with some extra packages.
* camera_values - perception node
* motion_controller - path tracking node

Extra details can be found in User_Manual_AAIP.pdf

## Run simulation
* run `roscore`
* run morse simulation, cd software_integration/morse/
   `morse run fourwd`

* run low level controller,  `roslaunch bear_car_launch myfile.launch `
  this file runs low level linear controller, low level steering controller, and hector slam with odometry   and lidar data.
  also this file will launch rviz file from /hector_slam/hector_slam_launch/rviz_cfg/mapping_demo.rviz

* run motion controller launch file: 
  `roslaunch move_controller motion_control.launch `
  this node will listen to published local path and will track it

* run perception module:
  `roslaunch camera_values camera_perception.launch `
   this file will campture the frames from camera, analyse them, take decision and publish desired local via points.

Run all command above in specified order, and the car should start drive automatically in the simulator

## Run real car

* run sensors on the car `roslaunch bear_car_launch sensors.launch`
* run localization with lidar  `roslaunch odometry_agent hectorslam_standalone.launch `
  this file will launch localization on the real car

* run motion controller launch file: 
  `roslaunch move_controller motion_control.launch `
  this node will listen to published local path and will track it

* run perception module, set is_simulation = false:
  `roslaunch camera_values camera_perception.launch is_simulation:=false`
   this file will campture the frames from camera, analyse them, take decision and publish desired local via points.


## To test the path tracking separately 
This file make the car follow some predefined path 
* To test in simulation run `rosrun camera_values followPath.py `
* To test on real car run `rosrun move_controller main_car.py `


## To test the astar separately 
To test in simulation,
* run `rosrun astar_planner astarPlanner.py`
* run `rosservice call /get_astar_path `
       while calling the service, provide the desired goal to generate a path
Note : The path is published under the topic `\astar_path`
