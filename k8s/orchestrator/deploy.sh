#!/bin/sh
kubectl apply -f orchestrator_service.yaml -n middleware
kubectl apply -f orchestrator_deployment.yaml -n middleware

