# Integrating 5G ERA ROS 2 Action Server with ROS1


This document assumes working with Summit XL, which is a robot which up to the current date
only supports by default native ROS 1. To include the ROS 2 Action server, a container running
ROS 2 Foxy with the Action Server will be used. Additionally, a container with ROS 1 Melodic
will be launched  with a version of ROS 1 Action Client. 
Therefore, by sending a action goal message via ROS 1 Action client, ROS2 action server in the
other container, will recieve the action and act accordingly. 
