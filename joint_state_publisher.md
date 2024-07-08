
# Controlling the robot arm by joint_state_publisher

Installing the package arduino_robot_arm

-- Add the “arduino_robot_arm” package to “src” folder



A

-$ cd ~/catkin_ws/src
	$ sudo apt install git

output:
[sudo] password for salwa: 
Reading package lists... Done
Building dependency tree       
Reading state information... Done
git is already the newest version (1:2.7.4-0ubuntu1.10).
0 upgraded, 0 newly installed, 0 to remove and 3 not upgraded.



	$ git clone https://github.com/smart-methods/arduino_robot_arm 

output :

fatal: destination path 'arduino_robot_arm' already exists and is not an empty directory

B- Install all the dependencies 
	$ cd ~/catkin_ws
	$ rosdep install --from-paths src --ignore-src -r -y

output: #All required rosdeps installed successfully





	$ sudo apt-get install ros-noetic-moveit


output:
Reading package lists... Done
Building dependency tree       
Reading state information... Done
E: Unable to locate package ros-noetic-moveit





	$ sudo apt-get install ros-noetic-joint-state-publisher ros-noetic-joint-state-publisher-gui

output :
Reading package lists... Done
Building dependency tree       
Reading state information... Done
E: Unable to locate package ros-noetic-joint-state-publisher
E: Unable to locate package ros-noetic-joint-state-publisher-gui

	$ sudo apt-get install ros-noetic-gazebo-ros-control joint-state-publisher

output:

Reading package lists... Done
Building dependency tree       
Reading state information... Done
E: Unable to locate package ros-noetic-gazebo-ros-control


	$ sudo apt-get install ros-noetic-ros-controllers ros-noetic-ros-control
output:Reading state information... Done
E: Unable to locate package ros-noetic-ros-controllers
E: Unable to locate package ros-noetic-ros-control

Compile the package
$ catkin_make

output:
Base path: /home/salwa/catkin_ws
Source space: /home/salwa/catkin_ws/src
Build space: /home/salwa/catkin_ws/build
Devel space: /home/salwa/catkin_ws/devel
Install space: /home/salwa/catkin_ws/install
####
#### Running command: "make cmake_check_build_system" in "/home/salwa/catkin_ws/build"
####
-- Using CATKIN_DEVEL_PREFIX: /home/salwa/catkin_ws/devel
-- Using CMAKE_PREFIX_PATH: /home/salwa/catkin_ws/devel;/opt/ros/kinetic
-- This workspace overlays: /home/salwa/catkin_ws/devel;/opt/ros/kinetic


$ roslaunch robot_arm_pkg check_motors.launch

output:
... logging to /home/salwa/.ros/log/6d9c9918-3d02-11ef-8ec9-0800278eeabe/roslaunch-salwa-VirtualBox-8788.log
Checking log directory for disk usage. This may take awhile.
Press Ctrl-C to interrupt
Done checking log file disk usage. Usage is <1GB.

started roslaunch server http://salwa-VirtualBox:32947/

SUMMARY
========

PARAMETERS
 * /robot_description: <?xml version="1....
 * /rosdistro: kinetic
 * /rosversion: 1.12.17

NODES
  /
    joint_state_publisher_gui (joint_state_publisher_gui/joint_state_publisher_gui)
    robot_state_publisher (robot_state_publisher/robot_state_publisher)
    rviz (rviz/rviz)

ROS_MASTER_URI=http://localhost:11311

process[robot_state_publisher-1]: started with pid [8808]
process[rviz-2]: started with pid [8809]
process[joint_state_publisher_gui-3]: started with pid [8810]
[INFO] [1720427432.364139]: Centering





![photo_2024-07-08_16-46-29](https://github.com/Salwa-Mhz/Ai-and-ROS/assets/172436383/7def7d28-cf6b-41ec-a966-1c31b73ecc45)








-Controlling the motors joint_state_publisher open new terminal 


$ rostopic echo /joint_states
output :header: 
  seq: 5544
  stamp: 
    secs: 1720427986
    nsecs: 879604101
  frame_id: ''
name: [base_joint, shoulder, elbow, wrist]
position: [0.0, 0.0, 0.0, 0.0]
velocity: []
effort: []
---



Controlling the motors in simulation

$ roslaunch robot_arm_pkg check_motors.launch



output:

... logging to /home/salwa/.ros/log/6d9c9918-3d02-11ef-8ec9-0800278eeabe/roslaunch-salwa-VirtualBox-10039.log
Checking log directory for disk usage. This may take awhile.
Press Ctrl-C to interrupt
Done checking log file disk usage. Usage is <1GB.

started roslaunch server http://salwa-VirtualBox:40161/

SUMMARY
========

PARAMETERS
 * /robot_description: <?xml version="1....
 * /rosdistro: kinetic
 * /rosversion: 1.12.17

NODES
  /
    joint_state_publisher_gui (joint_state_publisher_gui/joint_state_publisher_gui)
    robot_state_publisher (robot_state_publisher/robot_state_publisher)
    rviz (rviz/rviz)

ROS_MASTER_URI=http://localhost:11311

process[robot_state_publisher-1]: started with pid [10059]
process[rviz-2]: started with pid [10060]
process[joint_state_publisher_gui-3]: started with pid [10061]
[INFO] [1720428179.262254]: Centering



$ roslaunch robot_arm_pkg check_motors_gazebo.launch
 
output:
... logging to /home/salwa/.ros/log/6d9c9918-3d02-11ef-8ec9-0800278eeabe/roslaunch-salwa-VirtualBox-10391.log
Checking log directory for disk usage. This may take awhile.
Press Ctrl-C to interrupt
Done checking log file disk usage. Usage is <1GB.

started roslaunch server http://salwa-VirtualBox:40297/

SUMMARY
========

PARAMETERS
 * /arm_controller/gains/base_joint/d: 1
 * /arm_controller/gains/base_joint/i: 1
 * /arm_controller/gains/base_joint/i_clamp: 1
 * /arm_controller/gains/base_joint/p: 100
 * /arm_controller/gains/elbow/d: 1
 * /arm_controller/gains/elbow/i: 1
 * /arm_controller/gains/elbow/i_clamp: 1
 * /arm_controller/gains/elbow/p: 100
 * /arm_controller/gains/shoulder/d: 1
 * /arm_controller/gains/shoulder/i: 1
 * /arm_controller/gains/shoulder/i_clamp: 1
 * /arm_controller/gains/shoulder/p: 100
 * /arm_controller/gains/wrist/d: 1
 * /arm_controller/gains/wrist/i: 1
 * /arm_controller/gains/wrist/i_clamp: 1
 * /arm_controller/gains/wrist/p: 100
 * /arm_controller/joints: ['base_joint', 's...
 * /arm_controller/type: position_controll...
 * /joint_state_controller/publish_rate: 50
 * /joint_state_controller/type: joint_state_contr...
 * /robot_description: <?xml version="1....
 * /rosdistro: kinetic
 * /rosversion: 1.12.17
 * /use_sim_time: True

NODES
  /
    controller_spawner (controller_manager/spawner)
    gazebo (gazebo_ros/gzserver)
    gazebo_gui (gazebo_ros/gzclient)
    spawn_gazebo_model (gazebo_ros/spawn_model)

ROS_MASTER_URI=http://localhost:11311

process[gazebo-1]: started with pid [10410]
process[gazebo_gui-2]: started with pid [10415]
process[spawn_gazebo_model-3]: started with pid [10420]
process[controller_spawner-4]: started with pid [10421]
[INFO] [1720428285.519997, 0.000000]: Controller Spawner: Waiting for service controller_manager/load_controller
[ INFO] [1720428285.846498277]: Finished loading Gazebo ROS API Plugin.
[ INFO] [1720428285.852958976]: waitForService: Service [/gazebo/set_physics_properties] has not been advertised, waiting...
[ INFO] [1720428286.039842256]: Finished loading Gazebo ROS API Plugin.
[ INFO] [1720428286.041055096]: waitForService: Service [/gazebo/set_physics_properties] has not been advertised, waiting...
Error [parser.cc:581] Unable to find uri[model://sun]
SpawnModel script started
Error [parser.cc:581] Unable to find uri[model://ground_plane]
[INFO] [1720428287.799899, 0.000000]: Loading model XML from ros parameter
[INFO] [1720428287.824081, 0.000000]: Waiting for service /gazebo/spawn_urdf_model
[ INFO] [1720428288.078262281]: waitForService: Service [/gazebo/set_physics_properties] is now available.
[ INFO] [1720428288.080873417]: waitForService: Service [/gazebo/set_physics_properties] is now available.
[INFO] [1720428288.135848, 0.000000]: Calling service /gazebo/spawn_urdf_model
[INFO] [1720428288.738135, 0.001000]: Spawn status: SpawnModel: Successfully spawned entity
[ INFO] [1720428288.781590581, 0.001000000]: Physics dynamic reconfigure ready.
[ INFO] [1720428288.893777932]: Physics dynamic reconfigure ready.
[ INFO] [1720428289.015350832, 0.001000000]: Loading gazebo_ros_control plugin
[ERROR] [1720428289.016539905, 0.001000000]: GazeboRosControlPlugin missing <legacyModeNS> while using DefaultRobotHWSim, defaults to true.
This setting assumes you have an old package with an old implementation of DefaultRobotHWSim, where the robotNamespace is disregarded and absolute paths are used instead.
If you do not want to fix this issue in an old package just set <legacyModeNS> to true.

[ INFO] [1720428289.019155393, 0.001000000]: Starting gazebo_ros_control plugin in namespace: /
[ INFO] [1720428289.022934066, 0.001000000]: gazebo_ros_control plugin is waiting for model URDF in parameter [robot_description] on the ROS param server.
[spawn_gazebo_model-3] process has finished cleanly
log file: /home/salwa/.ros/log/6d9c9918-3d02-11ef-8ec9-0800278eeabe/spawn_gazebo_model-3*.log
[ INFO] [1720428289.179858361, 0.001000000]: Loaded gazebo_ros_control.
[ WARN] [1720428289.194984995, 0.002000000]: The default_robot_hw_sim plugin is using the Joint::SetPosition method without preserving the link velocity.
[INFO] [1720428289.196107, 0.002000]: Controller Spawner: Waiting for service controller_manager/switch_controller
[ WARN] [1720428289.196799391, 0.002000000]: As a result, gravity will not be simulated correctly for your model.
[ WARN] [1720428289.196943660, 0.002000000]: Please set gazebo_pid parameters, switch to the VelocityJointInterface or EffortJointInterface, or upgrade to Gazebo 9.
[ WARN] [1720428289.199179925, 0.002000000]: For details, see https://github.com/ros-simulation/gazebo_ros_pkgs/issues/612
[INFO] [1720428289.201262, 0.003000]: Controller Spawner: Waiting for service controller_manager/unload_controller
[INFO] [1720428289.204307, 0.004000]: Loading controller: arm_controller
[INFO] [1720428289.485665, 0.082000]: Controller Spawner: Loaded controllers: arm_controller
[INFO] [1720428289.511885, 0.083000]: Started controllers: arm_controller

![photo_2024-07-08_17-35-12](https://github.com/Salwa-Mhz/Ai-and-ROS/assets/172436383/c7d86619-ac6a-4eaf-a7c1-922a1283713a)

$ rosrun robot_arm_pkg joint_states_to_gazebo.py
output 
[rosrun] Couldn't find executable named joint_states_to_gazebo.py below /home/salwa/catkin_ws/src/arduino_robot_arm/robot_arm_pkg
[rosrun] Found the following, but they're either not files,
[rosrun] or not executable:
[rosrun]   /home/salwa/catkin_ws/src/arduino_robot_arm/robot_arm_pkg/scripts/joint_states_to_gazebo.py

You may need to change the permission 
	$ cd catkin/src/arduino_robot_arm/robot_arm_pkg/scripts

output 
bash: cd: catkin/src/arduino_robot_arm/robot_arm_pkg/scripts: No such file or directory

	$ sudo chmod +x joint_states_to_gazebo.py






