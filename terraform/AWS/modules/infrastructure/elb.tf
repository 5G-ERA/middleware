resource "aws_lb" "middldeware-lb" {
  name               = "middleware-lb"
  internal           = false
  load_balancer_type = "network"
  subnets            = module.vpc.private_subnets

  enable_deletion_protection = false

  tags = {
    terraform = "true"
  }
}

resource "aws_lb_target_group" "middleware-tg" {
  name     = "middleware-tg"
  port     = 80
  protocol = "TCP"
  vpc_id   = module.vpc.vpc_id
}

resource "aws_lb_listener" "front_end" {
  load_balancer_arn = aws_lb.middldeware-lb.arn
  port              = 80
  protocol          = "TCP"
  #ssl_policy        = "ELBSecurityPolicy-TLS13-1-2-2021-06"
  # TODO: create certificate
  #certificate_arn = ""

  default_action {
    type             = "forward"
    target_group_arn = aws_lb_target_group.middleware-tg.arn
  }
}

resource "aws_autoscaling_attachment" "middleware-asg-lb-attachment" {
  autoscaling_group_name = module.eks.eks_managed_node_groups_autoscaling_group_names[0]
  lb_target_group_arn    = aws_lb_target_group.middleware-tg.arn

  depends_on = [ module.eks ]
}