# demo-operator

## Description

This is basic minimal operator that the base for the future work. It contains the minimal required functions to work and be deployed properly.
This repository is created based on the default charm by Canonical created using the `charmcraft init` command.


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
The `demo-operator_ubuntu-20.04-amd64.charm` should be created. 

### Deploy 

To deploy the created charm, execute command

    juju deploy ./demo-operator_ubuntu-20.04-amd64.charm --resource redis-image=itzg/redis-server

### Upgrade

After some changes are made to the existing and deployed charm, it has be build once again and upgraded to the latest version with the following command:

    juju refresh demo-operator --path ./demo-operator_ubuntu-20.04-amd64.charm --resource redis-image=itzg/redis-server

### Remove

To remove the deployed charm use 

    juju remove-application demo-operator
## Relations

The following operator does not have any existing relations to other operators and can be used indecently.
## OCI Images

This demo operator utilizes the docker image of redis server. The original image is by [itzg](https://github.com/itzg/). The container is available on [DockerHub](https://registry.hub.docker.com/r/itzg/redis-server).

## Development

To develop own charms based on this demo charm, please follow instructions in the `CONTRIBUTING.md`.
