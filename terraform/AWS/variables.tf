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

variable "iam_policy_name" {
  type    = string
  default = "middleware-policy"
}

variable "iam_role_name" {

  type    = string
  default = "middleware-role"
}

variable "kubernetes_service_account_name" {
  description = "Name of teh service Account that the Middleware will use in k8s"
  type        = string
  default     = "orchestrator"
}