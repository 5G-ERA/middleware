# Integrating 5G ERA ROS 1 Action Server with Summit XL & NetApp


This document assumes working with Summit XL, a robot which up to the current day
only supports by default native ROS 1. To include the 5G ERA Action server, client and ROS 2 NetApp,
the inclusion of the ROS bridge and a complete ROS 1 implementation of 5g-era action server and client is available here.

### Running ROS 1 5G-ERA Action Client

```shell
docker-compose -f docker-compose-action-client-ros1.yml up
```
### Running ROS 1 5G-ERA Action Server

```shell
docker-compose -f docker-compose-action-server-ros1.yml up
```

___

Stopping the containers from the directory where the compose.yml are located:

```shell
docker-compose -f docker-compose-action-client-ros1.yml down
```


```shell
docker-compose -f docker-compose-action-server-ros1.yml down
```

## Rebuild the image from ROS1 directory:

```shell
docker build -f Dockerfile .
```

## Remove specific docker image

```shell
docker image rm docker_image_name
```

## Verification of process:
Please check if middleware is correctly deployed:
```shell
ubectl get all -n middleware  
```

```shell
Every 2,0s: kubectl get all -n middleware                                                                                                              summitxl-VirtualBox: Wed Jul 27 08:39:46 2022

NAME                                        READY   STATUS    RESTARTS   AGE
pod/orchestrator-api-5f4897c99c-qfzjq       1/1     Running   0          16m
pod/gateway-5bc57dd64b-brcjc                1/1     Running   0          5m9s
pod/redis-interface-api-7ffb94f698-pww2j    1/1     Running   0          5m7s
pod/resource-planner-api-7d686f85cf-xfchl   1/1     Running   0          5m7s
pod/task-planner-api-79895f6b45-qxmg6       1/1     Running   0          5m5s

NAME                           TYPE           CLUSTER-IP       EXTERNAL-IP    PORT(S)                      AGE
service/orchestrator-api       ClusterIP      10.152.183.232   <none>         80/TCP,433/TCP               16m
service/gateway                LoadBalancer   10.152.183.194   10.64.140.43   80:31145/TCP,443:32381/TCP   5m8s
service/redis-interface-api    ClusterIP      10.152.183.154   <none>         80/TCP,443/TCP               5m8s
service/resource-planner-api   ClusterIP      10.152.183.32    <none>         80/TCP,443/TCP               5m6s
service/task-planner-api       ClusterIP      10.152.183.57    <none>         80/TCP,443/TCP               5m5s

NAME                                   READY   UP-TO-DATE   AVAILABLE   AGE
deployment.apps/orchestrator-api       1/1     1            1           16m
deployment.apps/gateway                1/1     1            1           5m9s
deployment.apps/redis-interface-api    1/1     1            1           5m8s
deployment.apps/resource-planner-api   1/1     1            1           5m7s
deployment.apps/task-planner-api       1/1     1            1           5m6s

NAME                                              DESIRED   CURRENT   READY   AGE
replicaset.apps/orchestrator-api-5f4897c99c       1         1         1       16m
replicaset.apps/gateway-5bc57dd64b                1         1         1       5m9s
replicaset.apps/redis-interface-api-7ffb94f698    1         1         1       5m8s
replicaset.apps/resource-planner-api-7d686f85cf   1         1         1       5m7s
replicaset.apps/task-planner-api-79895f6b45       1         1         1       5m6s
```


## Common errors:

1) Docker middleware network not created.

```shell
Creating network "dockerera5gros1actionserverclient_ros" with driver "bridge"
ERROR: Network middleware_network declared as external, but could not be found. Please create the network manually using `docker network create middleware_network` and try again.
```

Solution:

```shell
docker network create middleware_network
```

2) K8 image pull backoff (watch -c kubectl get all -n middleware)

```shell
kubectl describe pod orchestrator-api-5f4897c99c-qfzjq -n middleware
```

```shell
Events:
  Type     Reason   Age                    From     Message
  ----     ------   ----                   ----     -------
  Normal   Pulling  4m3s (x4 over 5m33s)   kubelet  Pulling image "394603622351.dkr.ecr.eu-west-1.amazonaws.com/orchestrator-api:latest"
  Warning  Failed   4m2s (x4 over 5m30s)   kubelet  Failed to pull image "394603622351.dkr.ecr.eu-west-1.amazonaws.com/orchestrator-api:latest": rpc error: code = Unknown desc = failed to pull and unpack image "394603622351.dkr.ecr.eu-west-1.amazonaws.com/orchestrator-api:latest": failed to resolve reference "394603622351.dkr.ecr.eu-west-1.amazonaws.com/orchestrator-api:latest": pulling from host 394603622351.dkr.ecr.eu-west-1.amazonaws.com failed with status code [manifests latest]: 403 Forbidden
  Warning  Failed   4m2s (x4 over 5m30s)   kubelet  Error: ErrImagePull
  Warning  Failed   3m47s (x6 over 5m29s)  kubelet  Error: ImagePullBackOff
  Normal   BackOff  28s (x20 over 5m29s)   kubelet  Back-off pulling image "394603622351.dkr.ecr.eu-west-1.amazonaws.com/orchestrator-api:latest"
```

Solution: 

If error code is: 403 Forbidden, please authenticate docker with AWS credentials following official deployment of middleware documentation.


Note!
The token obtained using the aws ecr get-login-password will expire after a few hours and re-running
the command will be necessary, as Microk8s does not provide support for the credential-helpers. In
case when errors occur during the logging into the ecr, the deletion of the .docker/config.json file
usually helps.


