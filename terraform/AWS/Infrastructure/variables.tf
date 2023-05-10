# Copyright (c) HashiCorp, Inc.
# SPDX-License-Identifier: MPL-2.0

variable "region" {
  description = "AWS region"
  type        = string
  default     = "eu-west-2"
}

variable "cluster_name" {
  description = "AWS EKS Cluster name"
  type        = string
  default     = "5G-ERA-AWS"
}

variable "vpc_name" {
  description = "AWS VPC name"
  type        = string
  default     = "5G-ERA"
}

### Policy
variable "iam_policy_name" {
  type    = string
  default = "middleware-policy"
}

variable "role_name" {
  type    = string
  default = "middleware-role"
}