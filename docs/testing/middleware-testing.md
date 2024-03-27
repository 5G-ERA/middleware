middleware operating

orchestrator version was tested:
image: ghcr.io/5g-era/orchestrator-api:v1.0-198-beta6

relay_server_version in middleware: "but5gera/ros2_relay_server:1.3.0"

relay_client_version: ros2_relay_client_1.3.0

kubectl apply -n middleware -f orchestrator.yaml
kubectl -n middleware get all

kubectl delete deployment --all -n middleware
kubectl delete service --all -n middleware
kubectl -n middleware get all

etc/hosts in middleware must be updated/added with following config:
/etc/hosts
127.0.0.1       signal-quality.middleware.local
127.0.0.1       ros-object-detection-test.middleware.local
127.0.0.1       middleware.local


etc/hosts in the robot must be updated with following config
where 192.168.50.124 represent actual ip of middleware:


192.168.50.124       signal-quality.middleware.local
192.168.50.124       ros-object-detection-test.middleware.local
192.168.50.124       middleware.local

