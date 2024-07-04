# ROS Installation step by step:

1- Download VirtualBox by the link below:

( https://www.virtualbox.org/wiki/Downloads )

2- Download Ubuntu 16.04.7 LTS by the link below:

-( https://releases.ubuntu.com/16.04/)

3-set up VirtualBox:

A- open VirtualBox

B- Click new to create the virtual machine

C- rise the memory size as much as possible before the end of the green line. then next next ....

4- Click start ---> VirtualBox start

5- choose the drive file of Ubuntu you download, click install Ubuntu ,then next,next..... without change any settings.

6- choose your country,language

7- now Fill the next options, Username, create password, save it for later

8- go to Firefox in the main screen , put the link to get the programming commands to install Ros

(https://s-m.com.sa/ros.txt)

9- open terminal from the main screen , copy and paste the coomands ,then ros is installed.








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



2-To install and start the turtlesim open new terminal:


     $ sudo apt-get install ros-$(rosversion -d)-turtlesim


**the output** :  


Reading package lists... Done
Building dependency tree       
Reading state information... Done
ros-kinetic-turtlesim is already the newest version (0.7.1-0xenial-20210503-103643-0800).
ros-kinetic-turtlesim set to manually installed.
0 upgraded, 0 newly installed, 0 to remove and 3 not upgraded.

3-Run turtlesim:

        $ rosrun turtlesim turtlesim_node



**the output**:

$ rosrun turtlesim turtlesim_node
[ INFO] [1720035669.181655351]: Starting turtlesim with node name /turtlesim
[ INFO] [1720035669.188852436]: Spawning turtle [turtle1] at x=[5.544445], y=[5.544445], theta=[0.000000]


the turtlesim screen show up

4-turtle keyboard teleoperation
open new terminal and write down :

         $ rosrun turtlesim turtle_teleop_key

[ INFO] 1254264546.878445000: Started node [/teleop_turtle], pid [5528], bound on [aqy], xmlrpc port [43918], tcpros port [55936], logging to [~/ros/ros/log/teleop_turtle_5528.log], using [real] time
Reading from keyboard
---------------------------
Use arrow keys to move the turtle.


B-Using rostopic pub
-
-rostopic pub publishes data on to a topic currently advertised.


-  $ rostopic pub -1 /turtle1/cmd_vel geometry_msgs/Twist -- '[2.0, 0.0, 0.0]' '[0.0, 0.0, 1.8]' 
 
this command makes the turtle move in half circul path and stop

-to make it walk in continuous circular path 


   $ rostopic pub /turtle1/cmd_vel geometry_msgs/Twist -r 1 -- '[2.0, 0.0, 0.0]' '[0.0, 0.0, -1.8]'










