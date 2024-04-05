documentation relay client

robot can use 2 relay client methods:

1) Python: 
pip install era-5g-relay-network-application
sudo apt install ros-humble-rosbridge-library

export USE_MIDDLEWARE=true
export MIDDLEWARE_ADDRESS=middleware.local:30010
export MIDDLEWARE_USER=ad20f254-dc3b-406d-9f15-b73ccd47e867
export USER=ad20f254-dc3b-406d-9f15-b73ccd47e867
export MIDDLEWARE_PASSWORD=middleware
export MIDDLEWARE_TASK_ID=8017fc2e-4a30-43b7-a436-f2bd120ab54f
export MIDDLEWARE_ROBOT_ID=68013a37-7027-4b30-8c04-701debb0ba44
export TOPICS_TO_SERVER='[{"name": "/image_raw", "type": "sensor_msgs/msg/Image"}]'
export TOPICS_FROM_SERVER='[{"name": "/results", "type": "std_msgs/msg/String"}]'

python3 -m era_5g_relay_network_application.client

2) Docker engine (*not compatible with docker desktop)
docker run --network host --rm -e USE_MIDDLEWARE=true -e MIDDLEWARE_ADDRESS=middleware.local:30010 -e MIDDLEWARE_USER=ad20f254-dc3b-406d-9f15-b73ccd47e867 -e MIDDLEWARE_PASSWORD=middleware -e MIDDLEWARE_TASK_ID=8017fc2e-4a30-43b7-a436-f2bd120ab54f -e MIDDLEWARE_ROBOT_ID=68013a37-7027-4b30-8c04-701debb0ba44 -e TOPICS_TO_SERVER='[{"name": "/image_raw", "type": "sensor_msgs/msg/Image"}]' -e TOPICS_FROM_SERVER='[{"name": "/results", "type": "std_msgs/msg/String"}]' but5gera/ros2_relay_client:1.3.0

*optional (-e ROS_DOMAIN_ID=22)
