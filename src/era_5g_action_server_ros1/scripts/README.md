## Launch the action server:

* From the ROS workspace /home/dev_ws source the directory.

source /devel/setup.bash

* Add run permissions to the script if required.

chmod +x era_5g_action_server_script.py

* Manual run:

rosrun era_5g_action_server_ros1 era_5g_action_server_script.py 

## Expected output:

```shell
[INFO] [1658904524.417682]: Starting 5g-era-action-server...
[INFO] [1658904783.302266]: RECEIVED GOAL REQUEST - Trying to process request
[INFO] [1658904783.303697]: Feedback: index: 0
[INFO] [1658904783.305341]: Trying to log into the middleware
[INFO] [1658904783.930119]: Feedback: Login successful, token received
[INFO] [1658904783.931651]: Goal task is: c2a99272-8e84-4ae6-8c3d-1951d0985cdf
[INFO] [1658904799.386927]: PLAN: {u'Name': u'NetApp Standalone deployment', u'relations': [], u'TaskPriority': 3, u'ActionPlanId': u'73fed89f-1bd2-4a76-9d25-038ab08052ec', u'ActionSequence': [{u'ActionPriority': u'high', u'Name': u'move-base', u'ActionFamily': u'MAchine Learning Image Recognition', u'relations': [], u'Id': u'fe813ad8-0df5-4d03-9872-43b70c95dfaa', u'Services': [{u'ServiceType': u'Web-API', u'IsReusable': True, u'relations': [], u'ServiceUrl': None, u'DesiredStatus': u'running', u'ServiceStatus': None, u'Id': u'db7fcc94-3dad-4102-a82f-feb3d814308c', u'ServiceInstanceId': u'23e483da-1145-4141-b995-bb54a9d9243b', u'Name': u'ros-css-deployment'}], u'Placement': u'Edge_1', u'Order': 1}], u'Id': u'c2a99272-8e84-4ae6-8c3d-1951d0985cdf'}
[INFO] [1658904799.388561]: ActionPlanId ** is: [{u'ActionPriority': u'high', u'Name': u'move-base', u'ActionFamily': u'MAchine Learning Image Recognition', u'relations': [], u'Id': u'fe813ad8-0df5-4d03-9872-43b70c95dfaa', u'Services': [{u'ServiceType': u'Web-API', u'IsReusable': True, u'relations': [], u'ServiceUrl': None, u'DesiredStatus': u'running', u'ServiceStatus': None, u'Id': u'db7fcc94-3dad-4102-a82f-feb3d814308c', u'ServiceInstanceId': u'23e483da-1145-4141-b995-bb54a9d9243b', u'Name': u'ros-css-deployment'}], u'Placement': u'Edge_1', u'Order': 1}]
[INFO] [1658904799.389794]: ActionPlanId is: 73fed89f-1bd2-4a76-9d25-038ab08052ec
[INFO] [1658904799.390892]: Feedback: Got new plan successfully.
[INFO] [1658904799.392569]: SubactionRef 0
[INFO] [1658904799.394714]: SubactionRef <type 'int'>
[INFO] [1658904799.397052]: Executing New action goal... 
[INFO] [1658904799.398162]: type <type 'dict'>
[INFO] [1658904799.402266]: ActionSequenceIds: [u'fe813ad8-0df5-4d03-9872-43b70c95dfaa']
[INFO] [1658904799.403523]: Publishing first feedback
[INFO] [1658904799.404619]: Processing goal
[INFO] [1658904799.406214]: Looping query reosurce update
```
