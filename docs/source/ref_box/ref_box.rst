Refbox

In Robot

The robot should have fkie_master_discovery and fkie_master_sync 

Step 1: Run the master discovery node in the robot:

code-block:: bash
  rosrun fkie_master_discovery master_discovery

Starting the Refbox

**Step 1: Export the robot into local system in all terminals
code-block:: bash
  export ROS_MASTER_URI=http://<robot_ip>:11311

**Step 2: Start the __core__ and __com__ components:
code-block:: bash
  roslaunch atwork_commander atwork_commander.launch

for verbosity add "verbose:=true"
code-block:: bash
  roslaunch atwork_commander atwork_commander.launch verbose:=true

If the verbose is true terminal will show the status of the robot (it will in IDLE state before task generation)

**Step 3: Generate a task using the CLI:
code-block:: bash
  roslaunch atwork_commander generate.launch task:=<task to generate>

example tasks : BMT, RTT, BTT1 etc.

After task generation the robot will be in READY state.

**Step 4: Start the skynet in the robot 
code-block:: bash
  roslaunch mir_planning_core task_planning_sm.launch

After the skynet is started the generated task will be available in the robot under the topic 

code-block:: bash
  rostopic echo /atwork_commander/task


**Step 5: Start the task execution using the CLI:

code-block:: bash
  roslaunch atwork_commander execute.launch

