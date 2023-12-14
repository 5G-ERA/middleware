# TLS Configuration

The configuration for TLS has been implemented with <span style="color: yellow"> Cert Manager </span> and <span style="color: green"> Nginx Ingress Controller </span>.

## Setup <span style="color: yellow"> Cert-manager </span>

### Installing cert-manager with helm

### 1. Adding helm repository:

```
helm repo add jetstack https://charts.jetstack.io
```

### 2. Updating helm repository:

```
helm repo update
```

### 3. Installing Custom Resource Definitions CRD

```
kubectl apply -f https://github.com/cert-manager/cert-manager/releases/download/v1.13.3/cert-manager.crds.yaml
```

### 4. Installing Cert Manager

```
helm install cert-manager jetstack/cert-manager --namespace cert-manager --create-namespace --version v1.13.2
```


## Setup  <span style="color: green"> Nginx Ingress Controller </span>
