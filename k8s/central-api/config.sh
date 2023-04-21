#!/bin/sh
kubectl create ns middleware-central

kubectl apply -f central-api.yaml -n middleware-central
