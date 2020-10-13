## Six Wheeled Platform How to Run

### Connecting to the Jetson PC

Connect to the *CARS* wifi with password *Test1234*
Then, ssh terminals to Jetson PC:  

`ssh jetson3@192.168.1.66`
password for the jetson3: *jetsongtarc*

### Running the project

System is compromised of four basic components.

- Motor Controller Interface which takes cmd_vel message and translates it into motor commands.
- Odometry agent which initializes sensors and starts SLAM algorithms. Also publishes necessary transform trees.
- Local planner which calculates cmd_vel given odometry and path data
- Global planner which calculates the path from start to the end.

### *Starting Low Level Elements:*

Low level elements consist of three main nodes. These are ;

* serial_communicator: Starts the serial Communication between motor controller and Nvidia Jetson TX2.

  Which listens <u>/motor_controller/motor_commands</u> topic and publishes <u>/motor_controller/motor_controller_info</u>

* low_level_speed_controller: Stars the speed command generator.

  Which listens <u>/cmd_vel</u> topic and publishes <u>/output_speed</u>

* low_level_steering_controller: Starts the steering commands generator. This is only an interface which takes rad/s and translates it into servo commands.

  Which listens <u>/cmd_vel</u>  and /<u>speed_output</u> topics and publishes motor_controller/motor_commands</u>

To start them as a whole you can run the command:

`roslaunch bear_car_launch low_level_starter_sixwd.launch`

This command following commands:

`rosrun low_level_steering_controller six_wheel_low_level_steering_controller`

`rosrun low_level_speed_controller six_wheel_low_level_controller.launch`

`rosrun serial_6w serial_communicator_6w`



<u>/cmd_vel</u> is a physically appropriate message type which consists of geometry_msgs/Twist

<u>/serial_communicator/motor_commands</u> contains the motor control type and motor speeds in bytes

<u>/serial_communicator/motor_controller_info</u> contains motor speeds, battery voltage motor currents and temperature as bytes

##### *Starting Sensors:*

For now available sensors are

- 2D Lidar
- Camera IMU
- Wheel Encoder

  ##### To run them individually:
  
  Attention, required sensors are already included in the odometry agent launch file.

  ###### For Lidar Run:

  `roslaunch bear_car_launch rplidar_a3.launch`

  Which publishes /scan Topic and starts laser Scanner

  ###### For RGBD Camera Run:

  `roslaunch bear_car_launch rs_imu.launch`

  Which starts Realsense camera publishes /camera/imu data from Intel Realsense Camera then Filters it and Publishes again as /imu/data as sensor_msgs/Imu.


  ###### For Wheel Encoder Run:

  `roslaunch six_wheel_odom six_wheel_odom.launch`


##### *Starting Navigation stack step-by-step:*

Starting low-level components, as described above
`roslaunch bear_car_launch low_level_starter_sixwd.launch`

To start odometry and sensors (Lidar,Camera,IMU):

`roslaunch odometry_agent odometry_agent.launch is_four_wd:=false`

This package launches hector_slam, LIDAR driver and camera driver. All data from sensors are fused using robot_localization package. IMU data is retrieved from realsense camera.


To start all planners and pid controllers for them:

`roslaunch pose_follower navigation_stack_sixwd.launch`

This starts the controller node with parameters in param folder. By default this node listens to /cmd_vel and publishes commands with 50.0Hz and uses 4% duty cycle for takeoff. Topic names can be changed from the yaml file and control type, timeout and publish rate can be changed inside the node. Example setup can be found in nodes source code.

**TODO: Write the agent names comes with the roslaunch (and their brief descriptions)**

In cfg folder of pose_follower package also there is a rviz config for visualization. Pid parameters can be changed from pid_controller.launch file in pose_follower package. This launch file basically sets up move_base system with parameters and loads controllers.

To also start PID Controller but which is also contained in `navigation_stack_sixwd.launch`
`roslaunch pose_follower pid_controller_sixwd.launch`
**TODO: Write the agent names comes with the roslaunch (and their brief descriptions)**

To run in RC mode, DO NOT launch pose_follower diff_drive.launch:
`roslaunch ackermann_rc rc_driver_vesc.launch`

To run in RC mode with our low-level controller (integrated, but issues remain. See the open issues page), assign `true` to `use_twist` parameter under *ackermann_rc/param/rc_driver_vesc.yaml*, then run the rc_driver_vesc.launch above:

If you run in the Jetson PC directly (with a monitor), run rviz to control the car and its navigation:
`rosrun rviz rviz`
**TODO: here write how to add the config file**

 - WRITE Data that are available already with the config
 - How to give a goal (mention about the final orientation) -->
 - In cfg folder of pose_follower package also there is a rviz config for visualization. Pid parameters can be changed from pid_controller.launch file in pose_follower package. This launch file basically sets up move_base system with parameters and loads controllers.

##### Remote Control Mode (RC Mode)

For using the remote for controlling the car directly on low-level you only need the serial_6w package

`rosrun serial_6w serial_communicator_6w`

and `roslaunch ackermann_rc rc_six_wheel.launch`

Alternatively, you can directly launch the following launch file which includes above commands.

`roslaunch bear_car_launch direct_rc_sixwd.launch`

##### Emergency Stop

Kill switch is on the rear of car on the large motor controller board.

#### Interfacing

* TODO Services, topics their descriptions (high-level)
* TODO: global planner practical parameters (the ones that launch file imports). For all the rest provide the online links**
* TODO: local planner practical parameters, e.g. how to tune PID parameters, reaching goal point etc.**
* TODO: local cost map parameters (from move base)**
* TODO: how to send path arrays (overwriting the global planner)**
* TODO: odometry: Localization does fusion (how to change the weights of odometries?) and how to select the sensors/types/values from the launch file **

The odometry agent related interfaces are explained in thr readme inside the package.
