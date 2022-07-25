## Launch the action client:

* From the ROS workspace /home/dev_ws source the directory.

source /devel/setup.bash

* Add run permissions to the script if required.

chmod +x era_5g_action_client_script.py 


## Command line only action publisher:

Run first the action server 

rosrun era_5g_action_server_ros1 era_5g_action_server_script.py 

In another terminal, run:


rostopic pub /goal_5g/goal era_5g_action_interfaces_ros1/goal_5gActionGoal "header:
  seq: 0
  stamp:
    secs: 0
    nsecs: 0
  frame_id: ''
goal_id:
  stamp:
    secs: 0
    nsecs: 0
  id: ''
goal:
  goal_taskid: ''
  action_reference: 0" 
