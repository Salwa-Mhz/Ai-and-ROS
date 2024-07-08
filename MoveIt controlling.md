
# MoveIt controlling:


-$ roslaunch moveit_pkg demo.launch


output:
... logging to /home/salwa/.ros/log/185f9e9a-3d08-11ef-8ec9-0800278eeabe/roslaunch-salwa-VirtualBox-14561.log
Checking log directory for disk usage. This may take awhile.
Press Ctrl-C to interrupt
Done checking log file disk usage. Usage is <1GB.

started roslaunch server http://salwa-VirtualBox:34001/

SUMMARY
========

PARAMETERS
 * /joint_state_publisher/source_list: ['move_group/fake...
 * /joint_state_publisher/use_gui: False
 * /move_group/allow_trajectory_execution: True
 * /move_group/arm/default_planner_config: None
 * /move_group/arm/planner_configs: ['SBL', 'EST', 'L...
 * /move_group/capabilities: 
 * /move_group/controller_list: [{'joints': ['bas...
 * /move_group/disable_capabilities: 
 * /move_group/jiggle_fraction: 0.05
 * /move_group/max_range: 5.0
 * /move_group/max_safe_path_cost: 1
 * /move_group/moveit_controller_manager: moveit_fake_contr...
 * /move_group/moveit_manage_controllers: True
 * /move_group/octomap_resolution: 0.025
 * /move_group/planner_configs/BFMT/balanced: 0
 * /move_group/planner_configs/BFMT/cache_cc: 1
 * /move_group/planner_configs/BFMT/extended_fmt: 1
 * /move_group/planner_configs/BFMT/heuristics: 1
 * /move_group/planner_configs/BFMT/nearest_k: 1
 * /move_group/planner_configs/BFMT/num_samples: 1000
 * /move_group/planner_configs/BFMT/optimality: 1
 * /move_group/planner_configs/BFMT/radius_multiplier: 1.0
 * /move_group/planner_configs/BFMT/type: geometric::BFMT
 * /move_group/planner_configs/BKPIECE/border_fraction: 0.9
 * /move_group/planner_configs/BKPIECE/failed_expansion_score_factor: 0.5
 * /move_group/planner_configs/BKPIECE/min_valid_path_fraction: 0.5
 * /move_group/planner_configs/BKPIECE/range: 0.0
 * /move_group/planner_configs/BKPIECE/type: geometric::BKPIECE
 * /move_group/planner_configs/BiEST/range: 0.0
 * /move_group/planner_configs/BiEST/type: geometric::BiEST
 * /move_group/planner_configs/BiTRRT/cost_threshold: 1e300
 * /move_group/planner_configs/BiTRRT/frountier_node_ratio: 0.1
 * /move_group/planner_configs/BiTRRT/frountier_threshold: 0.0
 * /move_group/planner_configs/BiTRRT/init_temperature: 100
 * /move_group/planner_configs/BiTRRT/range: 0.0
 * /move_group/planner_configs/BiTRRT/temp_change_factor: 0.1
 * /move_group/planner_configs/BiTRRT/type: geometric::BiTRRT
 * /move_group/planner_configs/EST/goal_bias: 0.05
 * /move_group/planner_configs/EST/range: 0.0
 * /move_group/planner_configs/EST/type: geometric::EST
 * /move_group/planner_configs/FMT/cache_cc: 1
 * /move_group/planner_configs/FMT/extended_fmt: 1
 * /move_group/planner_configs/FMT/heuristics: 0
 * /move_group/planner_configs/FMT/nearest_k: 1
 * /move_group/planner_configs/FMT/num_samples: 1000
 * /move_group/planner_configs/FMT/radius_multiplier: 1.1
 * /move_group/planner_configs/FMT/type: geometric::FMT
 * /move_group/planner_configs/KPIECE/border_fraction: 0.9
 * /move_group/planner_configs/KPIECE/failed_expansion_score_factor: 0.5
 * /move_group/planner_configs/KPIECE/goal_bias: 0.05
 * /move_group/planner_configs/KPIECE/min_valid_path_fraction: 0.5
 * /move_group/planner_configs/KPIECE/range: 0.0
 * /move_group/planner_configs/KPIECE/type: geometric::KPIECE
 * /move_group/planner_configs/LBKPIECE/border_fraction: 0.9
 * /move_group/planner_configs/LBKPIECE/min_valid_path_fraction: 0.5
 * /move_group/planner_configs/LBKPIECE/range: 0.0
 * /move_group/planner_configs/LBKPIECE/type: geometric::LBKPIECE
 * /move_group/planner_configs/LBTRRT/epsilon: 0.4
 * /move_group/planner_configs/LBTRRT/goal_bias: 0.05
 * /move_group/planner_configs/LBTRRT/range: 0.0
 * /move_group/planner_configs/LBTRRT/type: geometric::LBTRRT
 * /move_group/planner_configs/LazyPRM/range: 0.0
 * /move_group/planner_configs/LazyPRM/type: geometric::LazyPRM
 * /move_group/planner_configs/LazyPRMstar/type: geometric::LazyPR...
 * /move_group/planner_configs/PDST/type: geometric::PDST
 * /move_group/planner_configs/PRM/max_nearest_neighbors: 10
 * /move_group/planner_configs/PRM/type: geometric::PRM
 * /move_group/planner_configs/PRMstar/type: geometric::PRMstar
 * /move_group/planner_configs/ProjEST/goal_bias: 0.05
 * /move_group/planner_configs/ProjEST/range: 0.0
 * /move_group/planner_configs/ProjEST/type: geometric::ProjEST
 * /move_group/planner_configs/RRT/goal_bias: 0.05
 * /move_group/planner_configs/RRT/range: 0.0
 * /move_group/planner_configs/RRT/type: geometric::RRT
 * /move_group/planner_configs/RRTConnect/range: 0.0
 * /move_group/planner_configs/RRTConnect/type: geometric::RRTCon...
 * /move_group/planner_configs/RRTstar/delay_collision_checking: 1
 * /move_group/planner_configs/RRTstar/goal_bias: 0.05
 * /move_group/planner_configs/RRTstar/range: 0.0
 * /move_group/planner_configs/RRTstar/type: geometric::RRTstar
 * /move_group/planner_configs/SBL/range: 0.0
 * /move_group/planner_configs/SBL/type: geometric::SBL
 * /move_group/planner_configs/SPARS/dense_delta_fraction: 0.001
 * /move_group/planner_configs/SPARS/max_failures: 1000
 * /move_group/planner_configs/SPARS/sparse_delta_fraction: 0.25
 * /move_group/planner_configs/SPARS/stretch_factor: 3.0
 * /move_group/planner_configs/SPARS/type: geometric::SPARS
 * /move_group/planner_configs/SPARStwo/dense_delta_fraction: 0.001
 * /move_group/planner_configs/SPARStwo/max_failures: 5000
 * /move_group/planner_configs/SPARStwo/sparse_delta_fraction: 0.25
 * /move_group/planner_configs/SPARStwo/stretch_factor: 3.0
 * /move_group/planner_configs/SPARStwo/type: geometric::SPARStwo
 * /move_group/planner_configs/STRIDE/degree: 16
 * /move_group/planner_configs/STRIDE/estimated_dimension: 0.0
 * /move_group/planner_configs/STRIDE/goal_bias: 0.05
 * /move_group/planner_configs/STRIDE/max_degree: 18
 * /move_group/planner_configs/STRIDE/max_pts_per_leaf: 6
 * /move_group/planner_configs/STRIDE/min_degree: 12
 * /move_group/planner_configs/STRIDE/min_valid_path_fraction: 0.2
 * /move_group/planner_configs/STRIDE/range: 0.0
 * /move_group/planner_configs/STRIDE/type: geometric::STRIDE
 * /move_group/planner_configs/STRIDE/use_projected_distance: 0
 * /move_group/planner_configs/TRRT/frountierNodeRatio: 0.1
 * /move_group/planner_configs/TRRT/frountier_threshold: 0.0
 * /move_group/planner_configs/TRRT/goal_bias: 0.05
 * /move_group/planner_configs/TRRT/init_temperature: 10e-6
 * /move_group/planner_configs/TRRT/k_constant: 0.0
 * /move_group/planner_configs/TRRT/max_states_failed: 10
 * /move_group/planner_configs/TRRT/min_temperature: 10e-10
 * /move_group/planner_configs/TRRT/range: 0.0
 * /move_group/planner_configs/TRRT/temp_change_factor: 2.0
 * /move_group/planner_configs/TRRT/type: geometric::TRRT
 * /move_group/planning_plugin: ompl_interface/OM...
 * /move_group/planning_scene_monitor/publish_geometry_updates: True
 * /move_group/planning_scene_monitor/publish_planning_scene: True
 * /move_group/planning_scene_monitor/publish_state_updates: True
 * /move_group/planning_scene_monitor/publish_transforms_updates: True
 * /move_group/request_adapters: default_planner_r...
 * /move_group/sensors: [{}]
 * /move_group/start_state_max_bounds_error: 0.1
 * /move_group/trajectory_execution/allowed_execution_duration_scaling: 1.2
 * /move_group/trajectory_execution/allowed_goal_duration_margin: 0.5
 * /move_group/trajectory_execution/allowed_start_tolerance: 0.01
 * /robot_description: <?xml version="1....
 * /robot_description_kinematics/arm/kinematics_solver: kdl_kinematics_pl...
 * /robot_description_kinematics/arm/kinematics_solver_attempts: 3
 * /robot_description_kinematics/arm/kinematics_solver_search_resolution: 0.005
 * /robot_description_kinematics/arm/kinematics_solver_timeout: 0.005
 * /robot_description_planning/joint_limits/base_joint/has_acceleration_limits: False
 * /robot_description_planning/joint_limits/base_joint/has_velocity_limits: True
 * /robot_description_planning/joint_limits/base_joint/max_acceleration: 0
 * /robot_description_planning/joint_limits/base_joint/max_velocity: 1
 * /robot_description_planning/joint_limits/elbow/has_acceleration_limits: False
 * /robot_description_planning/joint_limits/elbow/has_velocity_limits: True
 * /robot_description_planning/joint_limits/elbow/max_acceleration: 0
 * /robot_description_planning/joint_limits/elbow/max_velocity: 1
 * /robot_description_planning/joint_limits/shoulder/has_acceleration_limits: False
 * /robot_description_planning/joint_limits/shoulder/has_velocity_limits: True
 * /robot_description_planning/joint_limits/shoulder/max_acceleration: 0
 * /robot_description_planning/joint_limits/shoulder/max_velocity: 1
 * /robot_description_planning/joint_limits/wrist/has_acceleration_limits: False
 * /robot_description_planning/joint_limits/wrist/has_velocity_limits: True
 * /robot_description_planning/joint_limits/wrist/max_acceleration: 0
 * /robot_description_planning/joint_limits/wrist/max_velocity: 1
 * /robot_description_semantic: <?xml version="1....
 * /rosdistro: kinetic
 * /rosversion: 1.12.17
 * /rviz_salwa_VirtualBox_14561_4451717796384345965/arm/kinematics_solver: kdl_kinematics_pl...
 * /rviz_salwa_VirtualBox_14561_4451717796384345965/arm/kinematics_solver_attempts: 3
 * /rviz_salwa_VirtualBox_14561_4451717796384345965/arm/kinematics_solver_search_resolution: 0.005
 * /rviz_salwa_VirtualBox_14561_4451717796384345965/arm/kinematics_solver_timeout: 0.005

NODES
  /
    joint_state_publisher (joint_state_publisher/joint_state_publisher)
    move_group (moveit_ros_move_group/move_group)
    robot_state_publisher (robot_state_publisher/robot_state_publisher)
    rviz_salwa_VirtualBox_14561_4451717796384345965 (rviz/rviz)

ROS_MASTER_URI=http://localhost:11311

process[joint_state_publisher-1]: started with pid [14578]
process[robot_state_publisher-2]: started with pid [14579]
process[move_group-3]: started with pid [14580]
process[rviz_salwa_VirtualBox_14561_4451717796384345965-4]: started with pid [14582]
[ WARN] [1720429245.817562437]: The root link base has an inertia specified in the URDF, but KDL does not support a root link with an inertia.  As a workaround, you can add an extra dummy link to your URDF.
[ INFO] [1720429246.067127884]: Loading robot model 'arduino_robot_arm'...
[ WARN] [1720429246.309518658, 69.720000000]: Could not identify parent group for end-effector 'gripper'
[ INFO] [1720429246.441632035]: rviz version 1.12.17
[ INFO] [1720429246.447906144]: compiled against Qt version 5.5.1
[ INFO] [1720429246.448013918]: compiled against OGRE version 1.9.0 (Ghadamon)
[ INFO] [1720429246.470823432, 69.785000000]: Loading robot model 'arduino_robot_arm'...
[ WARN] [1720429246.537726046, 69.805000000]: Could not identify parent group for end-effector 'gripper'
[ WARN] [1720429246.542307021, 69.807000000]: The root link base has an inertia specified in the URDF, but KDL does not support a root link with an inertia.  As a workaround, you can add an extra dummy link to your URDF.
[ INFO] [1720429246.827067711, 69.961000000]: Publishing maintained planning scene on 'monitored_planning_scene'
[ INFO] [1720429246.872414337, 69.980000000]: MoveGroup debug mode is ON
Starting context monitors...
[ INFO] [1720429246.872491844, 69.980000000]: Starting scene monitor
[ INFO] [1720429246.926117456, 70.005000000]: Listening to '/planning_scene'
[ INFO] [1720429246.926199042, 70.005000000]: Starting world geometry monitor
[ INFO] [1720429246.974087792, 70.027000000]: Listening to '/collision_object' using message notifier with target frame '/base '
[ INFO] [1720429247.047416016, 70.046000000]: Listening to '/planning_scene_world' for planning scene world geometry
[ERROR] [1720429247.064884593, 70.051000000]: No sensor plugin specified for octomap updater 0; ignoring.
[ INFO] [1720429247.152018616, 70.087000000]: Stereo is NOT SUPPORTED
[ INFO] [1720429247.152228564, 70.087000000]: OpenGl version: 3 (GLSL 1.3).
[ INFO] [1720429247.183810525, 70.104000000]: Listening to '/attached_collision_object' for attached collision objects
Context monitors started.
[ INFO] [1720429247.297177788, 70.148000000]: Initializing OMPL interface using ROS parameters
[ERROR] [1720429247.350898444, 70.158000000]: Could not find the planner configuration 'None' on the param server
[ INFO] [1720429247.448035506, 70.215000000]: Using planning interface 'OMPL'
[ INFO] [1720429247.452599918, 70.215000000]: Param 'default_workspace_bounds' was not set. Using default value: 10
[ INFO] [1720429247.453370024, 70.215000000]: Param 'start_state_max_bounds_error' was set to 0.1
[ INFO] [1720429247.453957347, 70.215000000]: Param 'start_state_max_dt' was not set. Using default value: 0.5
[ INFO] [1720429247.454558160, 70.215000000]: Param 'start_state_max_dt' was not set. Using default value: 0.5
[ INFO] [1720429247.455116471, 70.215000000]: Param 'jiggle_fraction' was set to 0.05
[ INFO] [1720429247.455676347, 70.215000000]: Param 'max_sampling_attempts' was not set. Using default value: 100
[ INFO] [1720429247.455753492, 70.215000000]: Using planning request adapter 'Add Time Parameterization'
[ INFO] [1720429247.455790790, 70.215000000]: Using planning request adapter 'Fix Workspace Bounds'
[ INFO] [1720429247.455821401, 70.215000000]: Using planning request adapter 'Fix Start State Bounds'
[ INFO] [1720429247.455849985, 70.215000000]: Using planning request adapter 'Fix Start State In Collision'
[ INFO] [1720429247.455878700, 70.215000000]: Using planning request adapter 'Fix Start State Path Constraints'
[ INFO] [1720429247.511311792, 70.258000000]: Fake controller 'fake_arm_controller' with joints [ base_joint shoulder elbow wrist ]
[ INFO] [1720429247.513956682, 70.260000000]: Returned 1 controllers in list
[ INFO] [1720429247.574378008, 70.310000000]: Trajectory execution is managing controllers
Loading 'move_group/ApplyPlanningSceneService'...
Loading 'move_group/ClearOctomapService'...
Loading 'move_group/MoveGroupCartesianPathService'...
Loading 'move_group/MoveGroupExecuteTrajectoryAction'...
Loading 'move_group/MoveGroupGetPlanningSceneService'...
Loading 'move_group/MoveGroupKinematicsService'...
Loading 'move_group/MoveGroupMoveAction'...
Loading 'move_group/MoveGroupPickPlaceAction'...
Loading 'move_group/MoveGroupPlanService'...
Loading 'move_group/MoveGroupQueryPlannersService'...
Loading 'move_group/MoveGroupStateValidationService'...
[ INFO] [1720429247.863102192, 70.501000000]: 

********************************************************
* MoveGroup using: 
*     - ApplyPlanningSceneService
*     - ClearOctomapService
*     - CartesianPathService
*     - ExecuteTrajectoryAction
*     - GetPlanningSceneService
*     - KinematicsService
*     - MoveAction
*     - PickPlaceAction
*     - MotionPlanService
*     - QueryPlannersService
*     - StateValidationService
********************************************************

[ INFO] [1720429247.863224749, 70.501000000]: MoveGroup context using planning plugin ompl_interface/OMPLPlanner
[ INFO] [1720429247.863274597, 70.501000000]: MoveGroup context initialization complete

You can start planning now!

[ INFO] [1720429251.060279237, 73.363000000]: Loading robot model 'arduino_robot_arm'...
[ WARN] [1720429251.083290091, 73.370000000]: Could not identify parent group for end-effector 'gripper'
[ INFO] [1720429251.278212420, 73.489000000]: Loading robot model 'arduino_robot_arm'...
[ WARN] [1720429251.331509184, 73.525000000]: Could not identify parent group for end-effector 'gripper'
[ WARN] [1720429251.355434045, 73.530000000]: The root link base has an inertia specified in the URDF, but KDL does not support a root link with an inertia.  As a workaround, you can add an extra dummy link to your URDF.
[ INFO] [1720429251.767425074, 73.815000000]: Starting scene monitor
[ INFO] [1720429251.770472434, 73.815000000]: Listening to '/move_group/monitored_planning_scene'
[ WARN] [1720429252.051319597, 74.001000000]: The root link base has an inertia specified in the URDF, but KDL does not support a root link with an inertia.  As a workaround, you can add an extra dummy link to your URDF.
[ INFO] [1720429252.057728007, 74.005000000]: Constructing new MoveGroup connection for group 'arm' in namespace ''
[ INFO] [1720429253.298667577, 74.829000000]: Ready to take commands for planning group arm.
[ INFO] [1720429253.299845062, 74.829000000]: Looking around: no
[ INFO] [1720429253.300102860, 74.831000000]: Replanning: no
[ WARN] [1720429253.320359219, 74.839000000]: Interactive marker 'EE:goal_gripper' contains unnormalized quaternions. This warning will only be output once but may be true for others; enable DEBUG messages for ros.rviz.quaternions to see more details.


![photo_2024-07-08_17-00-16](https://github.com/Salwa-Mhz/Ai-and-ROS/assets/172436383/d8173993-b2ce-4301-a074-9aa6bf64de57)







$ roslaunch moveit_pkg demo_gazebo.launch 



output 







state_publisher-6] process has finished cleanly
log file: /home/salwa/.ros/log/44a7b4c8-3d09-11ef-8ec9-0800278eeabe/robot_state_publisher-6*.log
[robot_state_publisher-6] restarting process
process[robot_state_publisher-6]: started with pid [24630]
[ WARN] [1720446205.410411958]: The root link base has an inertia specified in the URDF, but KDL does not support a root link with an inertia.  As a workaround, you can add an extra dummy link to your URDF.
[ WARN] [1720446205.898897292]: Shutdown request received.
[ WARN] [1720446205.898930713]: Reason given for shutdown: [new node registered with same name]
[robot_state_publisher-6] process has finished cleanly


![photo_2024-07-08_16-56-42](https://github.com/Salwa-Mhz/Ai-and-ROS/assets/172436383/7319bdcb-b28e-423f-acc2-614793d98ac6)
