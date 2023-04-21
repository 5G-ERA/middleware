#!/bin/sh
kubectl delete deployment orchestrator-api -n middleware
kubectl delete service orchestrator-api -n middleware
