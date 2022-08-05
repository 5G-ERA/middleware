microk8s config > ~/.kube/config
rm ~/.docker/config.json
docker login -u AWS -p $(aws ecr get-login-password --region eu-west-1) 394603622351.dkr.ecr.eu-west-1.amazonaws.com
