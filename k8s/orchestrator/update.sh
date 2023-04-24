#!/bin/sh

kubectl delete deployment --all -n middleware

kubectl apply -f orchestrator.yaml -n middleware
