atwork-commander
================

.. image:: https://travis-ci.org/steup/atwork-commander.svg?branch=noetic
   :target: https://travis-ci.org/steup/atwork-commander

Complete reimplementation of the `old`_ Referee Box (Refbox) for the @Work-League of RoboCup.

This Refbox is a native ROS application.
However, it aims to enable multiple communication backends through individual plugins.
Additionally, a RViz GUI will be developed, which will provide enhanced visualization
capabilities for referees and visitors / spectators.
Generation of tasks conforming to the `rule book`_
are configurable using the ROS parameters in order to be easily adaptable for
any future changes in rules.

How to run the Refbox
=====================

Run in Robot
------------

Assumption:

- All other "bringup", "planning_bringup", "nav2d" and other(if available) components of the robot is running already
- Robot is localized

1. Run the master discovery node on the robot::

   rosrun fkie_master_discovery master_discovery

2. Listen to the /task topic in the robot::

   rostopic echo /atwork_commander/task

3. Run "Skynet" after generating the task from Local PC::

   roslaunch mir_planning_core task_planning_sm.launch

Run in Local PC
---------------

1. Run "roscore" in the local PC::

   roscore

2. Export the robot in two terminals in the local PC::

   export ROS_MASTER_URI=http://<robot_ip>:11311

   Warning: If the robot's roscore is not exported to the local terminal then the atwork_commander cannot change from 
   IDLE -> Ready state

3. Run "atwork_commander" in a terminal::

   roslaunch atwork_commander atwork_commander.launch

   Note: 
   - This terminal will be the primary monitoring place where all the stage changes can be monitored 
   - There is a --verbose command flag in the launch file, enable it to see the generated plan 

4. Generate plan::

   roslaunch atwork_commander generate.launch task:=<task to generate>

   Note:
   - Example tasks : BMT, RTT, BTT1 etc.
   - After task generation the robot will be in READY state.

5. Start "skynet" in the robot

6. Change State of robot from READY to PREPARATION::

   roslaunch atwork_commander start.launch

7. Change State of robot from PREPARATION to EXECUTION::

   roslaunch atwork_commander forward.launch

   Note: ROBOT should start moving after this step
