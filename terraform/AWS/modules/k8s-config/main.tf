locals {
  namespace = "middleware"
}

# Retrieve EKS cluster information
provider "aws" {
  region = var.region
}

data "aws_eks_cluster" "cluster" {
  name = var.cluster_name
}

provider "kubernetes" {
  host                   = var.cluster_endpoint
  cluster_ca_certificate = base64decode(var.cluster_ca_certificate)
  exec {
    api_version = "client.authentication.k8s.io/v1beta1"
    command     = "aws"
    args = [
      "eks",
      "get-token",
      "--cluster-name",
      var.cluster_name
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
      "eks.amazonaws.com/role-arn" = var.middleware_role_arn
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
    name = kubernetes_role.role.metadata[0].name
  }
  subject {
    kind = "ServiceAccount"
    namespace = local.namespace
    name = var.service_account_name
  }
}