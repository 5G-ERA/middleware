# redis-operator

## Description

The operator for the 5G-ERA Redis server for the use in the 5G-ERA project. 

It contains the required functions to work and be deployed properly and allow outher operators to communicate with this operator.
This repository is created based on the redis-k8s operator by Canonical.


## Usage

### Required components
To deploy the charm the following programs have to be installed

* Charmcraft
* Juju
* Juju bootstrapped cloud provider (AWS, AZURE or local Microk8s cluster)

The installation process for the work on the local environment can be found on the Juju [official website](https://juju.is/docs/sdk/dev-setup).

### Build
To build the charm, open the terminal and run the command:

```
charmcraft pack
```
The `redis-operator_ubuntu-20.04-amd64.charm` should be created. 

### Deploy 

To deploy the created charm, execute command

    juju deploy ./redis-operator_ubuntu-20.04-amd64.charm --resource redis-image=<address.for.the.5g-rea.redis.container>

### Upgrade

After some changes are made to the existing and deployed charm, it has be build once again and upgraded to the latest version with the following command:

    juju refresh redis-operator --path ./redis-operator_ubuntu-20.04-amd64.charm --resource redis-image=<address.for.the.5g-rea.redis.container>

### Remove

To remove the deployed charm use 

    juju remove-application demo-operator
## Relations

The following operator does not have any existing relations to other operators and can be used indecently.
## OCI Images

This operator utilizes the docker image of redis server that has built in modules for the RedisGraph and RedisJSON. The image for the link can be accessed on the AWS.

## Development

For development information, please follow instructions in the `CONTRIBUTING.md`.
