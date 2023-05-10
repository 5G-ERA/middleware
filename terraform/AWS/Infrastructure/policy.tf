resource "aws_iam_policy" "middleware_iam_policy" {
  name = var.iam_policy_name

  policy = jsonencode({
    Version = "2012-10-17"
    Statement = [
      {
        Effect = "Allow"
        Action = [
          "eks:AccessKubernetesApi",
          "secretsmanager:ListSecrets",
          "secretsmanager:GetSecretValue"
        ]
        Resource = "*"
      }
    ]
  })
}


data "aws_iam_policy_document" "middleware_assume_role_web_identity_policy" {
  statement {
    actions = ["sts:AssumeRoleWithWebIdentity"]
    effect  = "Allow"

    condition {
      test     = "StringEquals"
      variable = "${module.eks.oidc_provider}:sub"
      values   = ["system:serviceaccount:kube-system:aws-node"]
    }

    principals {
      identifiers = [aws_iam_openid_connect_provider.connection_provider.arn]
      type        = "Federated"
    }
  }
}


resource "aws_iam_role" "middleware_role" {
  name               = var.role_name
  assume_role_policy = data.aws_iam_policy_document.middleware_assume_role_web_identity_policy.json
}

# Attach the IAM policy to the IAM role
resource "aws_iam_policy_attachment" "middleware_role_policy_attachment" {
  name       = "Middleware Policy Attachement"
  policy_arn = aws_iam_policy.middleware_iam_policy.arn
  roles      = [aws_iam_role.middleware_role.name]
}

# resource "aws_iam_role" "middleware_role" {
#   name = var.role_name

#   assume_role_policy = jsonencode({
#     Version = "2012-10-17"
#     Statement = [
#       {
#         Effect = "Allow"
#         Principal = {
#           Service = "eks.amazonaws.com"
#         }
#         Action = "sts:AssumeRole"
#       }      
#     ]
#   })
# }