# Administrator documentation

This documentation section describes the necessary steps needed to deploy and maintain the healthy deployment of the Middleware.


## Key components

There are two key infrastructure components required to successfully start the 5G-ERA Middleware:
* Redis Cluster
* Central API deployment
* Middleware deployment

## Basic Infrastructure provision

In the case of provisioning the infrastructure in AWS Cloud, Middleware comes with the pre-prepared `Terraform` module that will deploy and configure the infrastructure. 

It will instantiate the following services:
* VPC with 3 public and private subnets
* EKS Cluster
* Network Load Balancer

On top of this, the EKS will be configured in a way that the Middleware can be deployed without any additional infrastructure configuration needed. 

Terraform module is located in the [terraform folder](../../terraform/AWS/README.md)