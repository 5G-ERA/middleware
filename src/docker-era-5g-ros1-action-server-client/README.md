# Integrating 5G ERA ROS 1 Action Server with Summit XL & NetApp


## 
This document assumes working with Summit XL, a robot which up to the current day
only supports by default native ROS 1. To include the 5G ERA Action server, client and ROS 2 NetApp,
the inclusion of the ROS bridge and a complete ROS 1 implementation of 5g-era action server and client is available here.

docker-compose -f docker-compose-action-client-ros1.yml up

docker-compose -f docker-compose-action-server-ros1.yml up

## rebuild the image from ROS1 directory:
docker build -f Dockerfile .

## Remove specific docker image

docker image rm docker_image_name
