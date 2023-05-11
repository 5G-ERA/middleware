# Copyright (c) HashiCorp, Inc.
# SPDX-License-Identifier: MPL-2.0

output "cluster_endpoint" {
  description = "Endpoint for EKS control plane"
  value       = module.eks.cluster_endpoint
}

output "cluster_security_group_id" {
  description = "Security group ids attached to the cluster control plane"
  value       = module.eks.cluster_security_group_id
}

output "region" {
  description = "AWS region"
  value       = var.region
}

output "cluster_name" {
  description = "Kubernetes Cluster Name"
  value       = module.eks.cluster_name
}

output "middleware_role_arn" {
  description = "ARN of the role that needs to be associated with the Service Account"
  value       = module.iam_eks_role.iam_role_arn
}

# output "middleware_address" {
#   description = "Fully qualified domain name under which the Middleware is accessible"
#   value       = aws_lb.middldeware-lb.dns_name
# }
