## Launch the action client:

* From the ROS workspace /home/dev_ws source the directory.

```shell
source /devel/setup.bash
```

* Add run permissions to the script if required.

```shell
chmod +x era_5g_action_client_script.py 
```

* Run the python script ros node

```shell
rosrun era_5g_action_server_ros1 era_5g_action_server_script.py 
```

## Command line only action publisher:

Run first the action server 

In another terminal, run:

```shell
cd /home/dev_ws
```

```shell
source devel/setup.bash
```

Distributed example task:
```shell
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
  goal_taskid: 'a9fdafe6-9b90-48bc-a286-3f08ed0a78aa'
  action_reference: 0
  resource_lock: false"
```


StandAlone example taskid: 
```shell
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
  goal_taskid: '72d9e9ee-e261-41f8-8135-1e5cc0db13f4'
  action_reference: 0
  resource_lock: false"
```


## Removing k8 deployed containers manually:

```shell
kubectl -n middleware delete deployment ros-css-deployment
```