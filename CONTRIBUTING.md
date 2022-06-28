# Contribution

This document describes the functionality needed to run and contribute to the 5G-ERA Middleware repository. This document assumes the machine is configured with the instructions in the [ENVIRONMENT.md](ENVIRONMENT.md) file.

## Development Environment configuration

The Middleware application is designed to be built around the Visual Studio 2022 installation utilizing the docker-compose support for the testing in the local development environment.
The following components have to be installed to successfully develop and test the applications in the DEV environment.

* **ROS Galactic** - currently Middleware supports only ROS Galactic due to the compatibility with the [5G-ERA NetApps](https://github.com/5G-ERA/Reference-NetApp)
* **Preferred .NET IDE** - Rider / Visual Studio Code / Visual Studio 2022 (for windows and WSL development)
* **Bridge to Kubernetes** - the Visual Studio / Visual Studio Code extension that allows to remotely debug applications running in the Kubernetes cluster. To install it, follow the [official guide](https://docs.microsoft.com/en-us/visualstudio/bridge/bridge-to-kubernetes-vs)

For the correct work of the Middleware, the Environment Variables have to be defined in the Operating system. Add 3 new Environment Variables if not defined already:

* **AWS_IMAGE_REGISTRY** - the address of the AWS Container Image Registry that hosts the official images for the 5G-ERA middleware

In addition, the Middleware has an option to log to centralized Elasticsearch. For this, the AWS SecretManager is used. The 2 environment variables have to be set to authenticate the machine against AWS.

* **AWS_ACCESS_KEY_ID** - AccessKeyId of the IAM user for the AWS account
* **AWS_SECRET_ACCESS_KEY** - Secret Access Key of the IAM user for the AWS account

For the AWS Secret Manager, the secrets are configured to be stored as plaintext and the names should have the following format: `Middleware-{SettingGroup}__{SettingValue}` for example `Middleware-Database__ConnectionString`.

The SecretManager will automatically map the required `appsettings.json` configuration sections into the correct data. The `{SettingGroup}` section represents the configuration group in the `appsettings.json` file. The `{SettingValue}` property allows to set the property value and the `__` is used to 


### Launching and debugging the application

To launch the application using `Bridge to Kubernetes`, the service with the backing pod has to be deployed. Example scripts are in the `k8s` folder of this repository and can be deployed using the commands below. The [orchestrator_deployment.yaml](/k8s/orchestrator/orchestrator_deployment.yaml) file has to be filled with the appropriate values for the environment variables specified in the file.

```shell
kubectl apply -f k8s/orchestrator/orchestrator_service.yaml -n middleware
kubectl apply -f k8s/orchestrator/orchestrator_deployment.yaml -n middleware
```

## Handle changes to the API definition

The code contains the internal references to the current definition of the API using the OpenAPI 3.0 specification schema. Based on this specification the code is automatically generated to create the clients for easier access to the API endpoints from the code.

The API schema specification needs to be updated after each change to the endpoints definition. Each change of the return value, status code, adding new endpoints and removing existing or the definition of the models has to be followed with the appropriate procedure described, to make sure that all the APIs are working as expected.

### Update API definition procedure

1. Launch the API in a standalone mode by selecting it as a startup project in the launch bar
2. Navigate to the Health Controller definition in the opened browser window.
3. Execute endpoint `api/health/spec` to download the latest information about the API by pressing the `Try it out` button`
4. Close browser window
5. Rebuild the whole solution to check for any errors across
6. Commit changes

## K8s cluster configuration 

To ensure the correct work of the Orchestrator, the cluster needs to have the specific configuration enabled. Orchestrator is managing the deployment, removal and listing of the Kubernetes services, deployments, pods and replica sets. For automating this it needs the required permissions. The permissions required are configured in the [k8s/cluster-config](k8s/cluster-config/) folder of this repository.

The required configuration creates the `ClusterRole` that allows to watch, list, deploy, and delete the resources in the namespaces. By default, the Orchestrator will work only in the `middleware` namespace of the cluster.

Other files are responsible for binding the `ClusterRole` to the namespace and creating the `ServiceAccount` that will allow to access the role from the Orchestrator deployment.


Orchestrator deployment must have the `ServiceAccount` specified in its deployment configuration. 

```yaml
apiVersion: apps/v1
kind: Deployment
metadata:
  name: orchestrator-api
spec:  
    ...
  template:    
    ...
    spec:
      serviceAccountName: orchestrator
      automountServiceAccountToken: true      
      containers:        
      ...
```