# TLS Configuration

The configuration for TLS has been implemented with `Cert Manager` and `Nginx Ingress Controller`.

## Setup Cert-manager
### Installing cert-manager with helm
### 1. Adding helm repository:

```
helm repo add jetstack https://charts.jetstack.io
```

### 2. Updating helm repository:

```
helm repo update
```

### 3. Installing Custom Resource Definitions CRD:

```
kubectl apply -f https://github.com/cert-manager/cert-manager/releases/download/v1.13.3/cert-manager.crds.yaml
```

### 4. Installing Cert Manager:

```
helm install cert-manager jetstack/cert-manager --namespace cert-manager --create-namespace --version v1.13.2
```


## Setup Nginx Ingress Controller
### Installing nginx with helm
### 1. Adding helm repository:

```
helm repo add ingress-nginx https://kubernetes.github.io/ingress-nginx
```

### 2. Updating helm repository:

```
helm repo update
```

### 3. Modify values.yaml file:
The nginx chart contains a `values.yaml` file, this file requires a few modifications in the `ipFamilyPolicy` section, in order to function with the current Middleware architecture. The initial configuration for the nginx controller service comes with `type: LoadBalancer`. Since the Middleware is already exposed through a Network Load Balancer this has to be adjusted to `type: NodePort`. In the Network Load Balancer that serves the Middleware system the listeners and target groups are assigned: `http: 80:31000` and `https: 443:31011` these properties also have to be adjusted to match the same values. The below code depicts the required modifications.   

```
    ipFamilyPolicy: "SingleStack"
    ipFamilies:
      - IPv4
    ports:
      http: 80
      https: 443
    targetPorts:
      http: http
      https: https
    type: NodePort
    nodePorts:
      http: 31000
      https: 31011
      tcp: {}
      udp: {}
    external:
      enabled: true
    internal:
```

### 4. Installing Nginx Ingress Controller:
To apply the modifications that were configured in section 3 above, when installing the nginx controller, specify the `-f values.yaml` in the installation command.

```
helm install ingress-nginx ingress-nginx/ingress-nginx --namespace ingress --create-namespace --set controller.ingressClassResource.name=nginx -f values.yaml
```

### 5. Check the service configuration:

```
kubectl -n ingress get svc
```

The desired result should look like below:
```
NAME                               TYPE       CLUSTER-IP       EXTERNAL-IP   PORT(S)                      AGE
service/ingress-nginx-controller   NodePort   172.20.190.113   <none>        80:31000/TCP,443:31011/TCP   11d
```

## Domain registration
The Middleware system has been registered under Amazon Route 53, and the TLS certificate authority used is Let's Encrypt. For other options for domain providers check the following list of compatibility with Let's Encrypt, under the following link:

```
https://community.letsencrypt.org/t/dns-providers-who-easily-integrate-with-lets-encrypt-dns-validation/86438
```



## Setup Ingress for Gateway and Central-API
First we setup the ingress for both services we want to access through the nginx controller without configuring the TLS yet.

### 1. Ingress for the Gateway
Create the ingress for the Gateway in a `ingress-gateway.yaml` file as below:

```
apiVersion: networking.k8s.io/v1
kind: Ingress
metadata:
  name: ingress-gateway
  namespace: middleware
  #This section will be uncommented after the cluster-issuer in the cert-manager is configured
  #annotations:
  #  cert-manager.io/cluster-issuer: "letsencrypt-prod"
spec:
  defaultBackend:
    service:
      name: gateway
      port:
        number: 80
  ingressClassName: nginx
  rules:
    - host: "*.5gera.net"
    - http:
        paths:
          - pathType: Prefix
            backend:
              service:
                name: gateway
                port:
                  number: 80
            path: /
  # This section will be uncommented after the cluster-issuer in the cert-manager is configured
  #tls:
  #  - hosts:
  #    - "*.5gera.net"
  #    secretName: tls-secret
```

Apply the yaml configuration file with: 

```
kubectl apply -f ingress-gateway.yaml
```

Check the ingress configuration with:

```
kubectl -n middleware get ingress
```
The desired result should look like below:
```
NAME              CLASS   HOSTS         ADDRESS          PORTS     AGE
ingress-gateway   nginx   *.5gera.net   172.20.190.113   80        11d
```

### 2. Ingress for the Central-API
Create the ingress for the Central-API in a `ingress-central-api.yaml` file as below:

```
apiVersion: networking.k8s.io/v1
kind: Ingress
metadata:
  name: ingress-central-api
  namespace: central-api
  #This section will be uncommented after the issuer in the cert-manager is configured
  #annotations:
  #  cert-manager.io/cluster-issuer: "letsencrypt-prod"
spec:
  defaultBackend:
    service:
      name: central-api
      port:
        number: 80
  ingressClassName: nginx
  rules:
    - host: central-api.5gera.net
      http:
        paths:
          - pathType: Prefix
            backend:
              service:
                name: central-api
                port:
                  number: 80
            path: /
  # This section will be uncommented after the issuer in the cert-manager is configured
  #tls:
  #  - hosts:
  #    - central-api.5gera.net
  #    secretName: tls-secret
```
Apply the yaml configuration file with: 

```
kubectl apply -f ingress-central-api.yaml
```

Check the ingress configuration with:

```
kubectl -n central-api get ingress
```
The desired result should look like below:
```
NAME                  CLASS   HOSTS                   ADDRESS          PORTS   AGE
ingress-central-api   nginx   central-api.5gera.net   172.20.190.113   80      11d
```
## Setup Cert Manager Issuer to obtain the TLS certificate

## Securing the Ingress with TLS certificate