# Environment 
This document shows the process of configuring the testing environment for running and testing the 5G-ERA Middleware. It also has the required files for the configuration of the Kubernetes cluster to accommodate the functionality needed by the middleware.

The testing environment is based on the `Microk8s`, the minimal Kubernetes installation. It assumes the use of the `Ubuntu 20.04` operating system. For other Linux distributions or other operating systems, the instructions may vary, so check the official guides for installing the respective software. 

Recommended hardware:  
* Processor - 8 logical cores or above. 
* Ram - 16GB or above. 
* Storage - 100GB free space.

## Docker Engine installation

The first step is to install Docker Engine. Docker Engine is used for the verification of the credentials and pulling the containers from the private AWS registry that Middleware uses.
To install the Docker Engine, refer to [Docker's official website](https://docs.docker.com/engine/install/ubuntu/) on how to install it on the Ubuntu distro.
For easier access to the Docker CLI execute the [post-installation steps](https://docs.docker.com/engine/install/linux-postinstall/).

## AWS CLI installation
After the installation of the Docker, the AWS CLI must be installed. It is used to authenticate the computer and the Docker client for access to the private container registries.
To install the AWS CLI, follow the [official guide](https://docs.aws.amazon.com/cli/latest/userguide/getting-started-install.html) on the installation process.
After finishing, the installation process of AWS CLI it is time to configure the AWS CLI with the IAM account that has access to the services needed to run the Middleware.

Use the following command to configure the AWS CLI:
```shell
$ aws configure
```

During the execution of the command supply the AWS Access Key ID and AWS Secret Access Key and the default region to be used. The required container registry is in the `eu-west-1` region. You can also specify the default output format for the CLI.

## Authenticate docker with AWS credentials

After the Docker and the AWS CLI are installed, use the following command to authenticate your device against the AWS cloud.
```shell
$ docker login -u AWS -p $(aws ecr get-login-password --region eu-west-1) 394603622351.dkr.ecr.eu-west-1.amazonaws.com
```
After executing this command, you should be able to pull the images from the private AWS registry. It can be verified using the following command:
```shell
$ sudo docker pull 394603622351.dkr.ecr.eu-west-1.amazonaws.com/5g-era-redis
```
This will download the `5g-era-redis` image that hosts the Redis cache.

### Note!

The token obtained using the `aws ecr get-login-password` command will expire after a few hours and re-running the command will be necessary, as Microk8s does not provide support for the credential-helpers. In case when errors occur during the logging into the `ecr`, the deletion of the .docker/config.json file usually helps.

## Install kubectl

The `kubectl` is the command-line tool that allows communication and management of the Kubernetes cluster. To install it use the preferred way on the [official guide](https://kubernetes.io/docs/tasks/tools/install-kubectl-linux/).

## Install Microk8s
Microk8s is the minimal Kubernetes installation that can be used on the local computer. It will be used to run the Middleware. Install it with the command:
```shell
$ sudo snap install microk8s --classic
$ sudo usermod -a -G microk8s $USER 
$ sudo chown -f -R $USER ~/.kube
```

After the installation is finished copy the configuration file of the Microk8s to the `.kube/config` file so the `kubectl` command can communicate with the newly installed cluster.

```shell
$ microk8s config > ~/.kube/config
```
Afterwards, validate the connection to the cluster with the command
```shell
$ kubectl get all -n kube-system
```

Afterwards, the additional modules for the microk8s must be installed:
```
$ sudo microk8s enable dns multus 
$ sudo microk8s enable ingress metallb 
```

It enables the DNS on the cluster as well as the Load Balancer and Multus network card. During the installation, the program will ask for the range of the IP addresses for the Load Balancer. Provide desired range, it can be a default one.

## Configure Microk8s to access Docker credentials

After the successful installation of the Microk8s, it must be configured for access to the private AWS Registry. For this, the Microk8s must be stopped. 
```
$ microk8s stop
```
In the next step create the link to the Docker credentials for the Microk8s:
```shell
$ sudo ln -s ~/.docker/config.json \
  /var/snap/microk8s/common/var/lib/kubelet/
```
Next, start the Microk8s
```shell
$ microk8s start
```
Afterwards, Microk8s is operational and can clone the images from the private 5G-ERA repositories.

## Microk8s cluster configuration

After the Microk8s is installed and the `kubectl` command has access to the cluster, it is time to configure the cluster so the middleware can be deployed and function correctly inside of it.

For this purpose, the Service Account with the correct permissions is needed. The Service Account will give the necessary permissions for the Middleware access to the Kubernetes API and to manage the resources as a part of its functionality. Additionally, the Multus extension has to be configured to enable the additional Network Interface Card on the pods that require communication through ROS protocols.

### Multus configuration

The [multus_config.yaml](/k8s/cluster-config/multus_config.yaml) has to be adjusted to reflect the real state of the network configuration in the testing environment. 

A few properties in the file must be changed:
* `spec -> config -> master` – has to be set with the real name of the NIC with access to the internet in the testing environment. Could be retrieved by using the `ifconfig` command. The example values are: `eth0`, `enp0s3`.
* `spec -> config -> ipam -> ranges` – the ranges of the IP addresses must be updated to reflect the real subnet of the NIC selected in the previous point. The ranges will specify how many pods will be able to use this network.
The file in the appendix has to be treated as an example and has to be modified to”, and “enp0s3”. testing environment network configuration.

To apply the prepared file use the following command: 
```shell
$ kubectl apply -f multus_config.yaml
```
### Middleware namespace

As the whole middleware operates in the middleware k8s namespace, it is required to create it before the launch of the service. To do so, use the command:
```
$ kubectl create namespace middleware
```
Using the [orchestrator_service_account.yaml](k8s/cluster-config/orchestrator_service_account.yaml) file, create the Service Account with the following command:
```shell
$ kubectl apply -f orchestrator_service_account.yaml  
```

The next step is to create the `Cluster Role`, which specifies the permissions needed for the proper functioning of the Middleware. `Cluster Role` specifies the permissions to get, watch, list create and delete resources in the Middleware namespace. It affects the pods, services, deployments, namespaces, and replica sets in the cluster. To create the `Cluster` Role`, use the [orchestrator_cluster_role.yaml](k8s/cluster-config/orchestrator_cluster_role.yaml) file with the following command:
```shell
$ kubectl apply -f orchestrator_cluster_role.yaml
```
The last step to configuring the Kubernetes cluster is to bind the Cluster Role to the Service Account. For this the `Cluster Role Binding` is necessary. To create it, use the [orchestrator_cluster_role_binding.yaml](k8s/cluster-config/orchestrator_cluster_role_binding.yaml) file with the following command:
```
$ kubectl apply -f orchestrator_cluster_role_binding.yaml
```

## Middleware deployment
The last step is to prepare the deployment script for the middleware. In the [orchestrator_deployment.yaml](k8s/orchestrator/orchestrator_deployment.yaml) file there are environment variables that must be set for the correct work of the Orchestrator. The needed variables are:
1.	AWS_IMAGE_REGISTRY – contains the address of the registry in which the Middleware images are stored
4.	REDIS_INTERFACE_ADDRESS – address on which the API client for the REDIS in the middleware operates. Defaults to http://redis-interface-api

After all the values are set, the Middleware can be deployed. Start with the deployment of the service for the Orchestrator:
```shell
$ kubectl apply -n middleware -f orchestrator_service.yaml
```
Afterwards, deploy the Orchestrator Deployment:
```shell
$ kubectl apply -n middleware -f orchestrator_deployment.yaml
```

The containers will be downloaded, and the Orchestrator will deploy the rest of the Middleware deployments and services required to function correctly. 

## Verification of Middleware Deployment

To check and monitor the status of the deployment of the Middleware services use the following command 
```shell
$ watch -c kubectl get all -n middleware
```
It will monitor the status of all the services deployed in the middleware namespace. The following objects should be deployed:
1.	Orchestrator
2.	Redis interface
3.	Gateway
4.	Task planner
5.	Resource planner

Each of these services is represented by the pod, service, deployment and replica set in the Kubernetes environment. With the deployment of the Orchestrator, the other services are deployed automatically. The process of their deployment may take a while depending on the internet connection that machine has. If only the Orchestrator is visible with the status of the pod as Container Creating, it needs additional time to download the application. 

After the deployment of the Orchestrator, soon the other components should begin their deployment. The result should look like the one presented below.

<p align="center">
  <img src="docs/img/DeployedMiddleware.png" />
</p>

If there are errors during the deployment of the Orchestrator, then check if you correctly configured access to the AWS registry. 
In case there are any errors during the deployment of the Gateway and Redis interface, check if the firewall does not block access to the Redis server.

After the deployment is complete the gateway should be accessible through the IP address specified in the `EXTERNAL-IP` column. 

In case the IP address is not working use the following command to redirect the traffic from the specified port on the localhost to the gateway:
```shell
$ kubectl port-forward -n middleware service/gateway 5000:80
```
This command will port forward the traffic from port 5000 to port 80 in the service. The middleware will be now accessible under the following address:

> http://localhost:5000
