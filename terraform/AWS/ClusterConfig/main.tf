data "terraform_remote_state" "eks" {
  backend = "local"

  config = {
    path = "../Infrastructure/terraform.tfstate"
  }
}

locals {
  namespace = "middleware"
}

# Retrieve EKS cluster information
provider "aws" {
  region = data.terraform_remote_state.eks.outputs.region
}

data "aws_eks_cluster" "cluster" {
  name = data.terraform_remote_state.eks.outputs.cluster_name
}

provider "kubernetes" {
  host                   = data.aws_eks_cluster.cluster.endpoint
  cluster_ca_certificate = base64decode(data.aws_eks_cluster.cluster.certificate_authority.0.data)
  exec {
    api_version = "client.authentication.k8s.io/v1beta1"
    command     = "aws"
    args = [
      "eks",
      "get-token",
      "--cluster-name",
      data.aws_eks_cluster.cluster.name
    ]
  }
}

resource "kubernetes_namespace" "middleware" {
  metadata {
    name = local.namespace
    annotations = {
      name = local.namespace
    }
  }
  
}

resource "kubernetes_service_account" "orchestrator" {

  metadata {
    namespace = local.namespace
    name      = var.service_account_name
    annotations = {
      "eks.amazonaws.com/role-arn" = data.terraform_remote_state.middleware_role_arn
    }
  }
  automount_service_account_token = true
}

resource "kubernetes_role" "role" {  
  metadata {
    name      = "${var.service_account_name}-role"
    namespace = local.namespace
  }
  rule {
    api_groups = ["", "apps", "batch"]
    resources = ["services",
      "nodes",
      "namespaces",
      "pods",
      "deployments",
      "daemonsets",
      "statefulsets",
      "replicasets",
      "jobs",
      "pods/log",
    "pods/exec"]
    verbs = ["get", "list", "watch", "create", "delete"]
  }
}

resource "kubernetes_role_binding" "role_binding" {
  metadata {
    name = "${var.service_account_name}-role-binding"
    namespace = local.namespace
  }
  role_ref {
    api_group = "rbac.authorization.k8s.io"
    kind = "Role"
    name = local.namespace
  }
  subject {
    kind = "ServiceAccount"
    namespace = local.namespace
    name = var.service_account_name
  }
}