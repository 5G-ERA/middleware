#!/bin/sh
kubectl port-forward -n middleware service/gateway 5000:80
