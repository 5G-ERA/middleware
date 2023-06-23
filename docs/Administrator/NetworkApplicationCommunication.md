# Network Application Communication

This document explains how Middleware enables communication between multiple network applications that have to cooperate. 

## Deployment-based communication enablement

By default, when Middleware deploys Network Applications, it enables basic communication between them. During the deployment, each `Task` is constructed using `Actions`. An `Action` is a singular piece of service, that allows the robot to conduct a specific function, for example, navigation and mapping. 

When Middleware deploys an `Action` that consists of multiple `Instances` each of the instances is informed about the existence of others. By setting the environment variables, the Middleware gives the addresses by which other Network Applications can be accessed. 

The environment variables are named exactly as the Network Applications they reference, but in uppercase and `_` instead of `-`. Each environment variable points to the Kubernetes Service that exposes the deployment. 

When the Network Application is configured by the user not to be exposed by a service, Middleware will create a default Kubernetes Service of type `ClusterIP` to enable communication using default `http` and `https` ports.

## Communication of ROS-based Network Applications

To be done with [#158](https://github.com/5G-ERA/middleware/issues/158)