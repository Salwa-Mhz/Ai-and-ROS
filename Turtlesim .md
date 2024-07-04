# A-Getting Started with Turtlesim

1-Start the roscore:

    $ roscore

**the output** :

... logging to /home/salwa/.ros/log/87f07080-3971-11ef-8e17-0800278eeabe/roslaunch-salwa-VirtualBox-2062.log
Checking log directory for disk usage. This may take awhile.
Press Ctrl-C to interrupt
Done checking log file disk usage. Usage is <1GB.

started roslaunch server http://salwa-VirtualBox:34257/
ros_comm version 1.12.17


SUMMARY
========

PARAMETERS
 * /rosdistro: kinetic
 * /rosversion: 1.12.17

NODES

auto-starting new master
process[master]: started with pid [2073]
ROS_MASTER_URI=http://salwa-VirtualBox:11311/

setting /run_id to 87f07080-3971-11ef-8e17-0800278eeabe
process[rosout-1]: started with pid [2086]
started core service [/rosout]
.
.
.
.
.



2-To install and start the turtlesim open new terminal:


     $ sudo apt-get install ros-$(rosversion -d)-turtlesim


**the output** :  


Reading package lists... Done
Building dependency tree       
Reading state information... Done
ros-kinetic-turtlesim is already the newest version (0.7.1-0xenial-20210503-103643-0800).
ros-kinetic-turtlesim set to manually installed.
0 upgraded, 0 newly installed, 0 to remove and 3 not upgraded.

.
.
.
.


3-Run turtlesim:

        $ rosrun turtlesim turtlesim_node



**the output**

$ rosrun turtlesim turtlesim_node

[ INFO] [1720035669.181655351]: Starting turtlesim with node name /turtlesim
[ INFO] [1720035669.188852436]: Spawning turtle [turtle1] at x=[5.544445], y=[5.544445], theta=[0.000000]


**the turtlesim window show up**
![photo_2024-07-04_15-19-30](https://github.com/Salwa-Mhz/Ai-and-ROS/assets/172436383/7d45e83a-8c4c-4118-a99f-809288d64f22)

.
.
.
.

4-turtle keyboard teleoperation
open new terminal and write down :

         $ rosrun turtlesim turtle_teleop_key

[ INFO] 1254264546.878445000: Started node [/teleop_turtle], pid [5528], bound on [aqy], xmlrpc port [43918], tcpros port [55936], logging to [~/ros/ros/log/teleop_turtle_5528.log], using [real] time
Reading from keyboard
---------------------------
Use arrow keys to move the turtle.

.
.
.
.
.

B-Using rostopic pub

-rostopic pub publishes data on to a topic currently advertised.


    $ rostopic pub -1 /turtle1/cmd_vel geometry_msgs/Twist -- '[2.0, 0.0, 0.0]' '[0.0, 0.0, 1.8]' 
 
this command makes the turtle move in half circul path and stop
![photo_2024-07-04_15-19-47](https://github.com/Salwa-Mhz/Ai-and-ROS/assets/172436383/32b770fc-b0fe-4c1e-a9de-b5ad34aed0e6)

-to make it walk in continuous circular path 


       $ rostopic pub /turtle1/cmd_vel geometry_msgs/Twist -r 1 -- '[2.0, 0.0, 0.0]' '[0.0, 0.0, -1.8]'


![photo_2024-07-04_15-11-s](https://github.com/Salwa-Mhz/Ai-and-ROS/assets/172436383/48d0e576-0b59-4a2f-bda5-030c8aed713e)










