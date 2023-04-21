# AWS cloud-config for 5G-ERA Middleware


The process of configuring the AWS EKS cluster to run the 5G-ERA Middleware consists of a few steps. 
This guide assumes that the AWS EKS Cluster is already created. The guide also assumes that on the machine the `AWS CLI` and `eksctl` are configured.

1. [Setting up the user pool](#setting-up-the-user-pool)
2. [Create an IAM OIDC provider](#create-an-iam-oidc-provider)
3. [Service Account configuration](#service-account-configuration)
4. [Trust relationships for the IAM role](#trust-relationships-for-the-iam-role)


---

## Setting up the user pool
To configure your identity pool

1. Open the [Amazon Cognito console](https://console.aws.amazon.com/cognito/home). If prompted, enter your AWS credentials.

2. Choose Manage Identity Pools.

3. Choose the name of the identity pool for which you want to enable Amazon Cognito user pools as a provider.

4. On the Dashboard page, choose Edit identity pool.

5. Expand the Authentication providers section.

6. Choose Cognito.

7. Enter the User Pool ID.

8. Enter the App Client ID. This must be the same client app ID that you received when you created the app in the Your User Pools section of the AWS Management Console for Amazon Cognito.

9. If you have additional apps or user pools, choose "Add Another Provider" and enter the User Pool ID and App Client ID for each app in each user pool.

10. When you have no more apps or user pools to add, choose Save Changes. If successful, you will see a Changes saved successfully message on the Dashboard page.


## Create an IAM OIDC provider

To create an IAM OIDC identity provider for your cluster with eksctl

1. Determine whether you have an existing IAM OIDC provider for your cluster.

    View your cluster's OIDC provider URL. Replace the `my-cluster` with the correct name of the cluster.
    ``` bash
    aws eks describe-cluster --name my-cluster --query "cluster.identity.oidc.issuer" --output text
    ```
    The example output should look like the following:
    ```bash
    https://oidc.eks.region-code.amazonaws.com/id/EXAMPLED539D4633E53DE1B71EXAMPLE
    ```
    List the IAM OIDC providers in your account. Replace `EXAMPLED539D4633E53DE1B71EXAMPLE` with the value returned from the previous command.
    ```bash
    aws iam list-open-id-connect-providers | grep EXAMPLED539D4633E53DE1B71EXAMPLE
    ```
    If the output is returned from this command, the OIDC provider is already configured for the cluster. 
2. Create an IAM OIDC identity provider for your cluster with the following command. Replace `my-cluster` with the name of the cluster.
    ```bash
    eksctl utils associate-iam-oidc-provider --cluster my-cluster --approve
    ```
## Create Role and Policy
The IAM Role will be used for giving the pod the necessary permissions to deploy the Middleware. The Role will have to have the necessary permissions to integrate with the EKS as well as EC2.

## Service account configuration

Create the Service Account and the Role, as described in the definition of preparing the testing environment. 

On top of this, the service account has to be annotated with the `arn` of the IAM Role. 
Do this with the following command replacing `service-account-name` with the name of the service account that will be annotated and the `arn:aws:iam::111122223333:role/iam-role-name` with the correct `arn` of the role you gave the necessary permissions:

```bash
kubectl annotate serviceaccount -n service-account-namespace service-account-name \
eks.amazonaws.com/role-arn=arn:aws:iam::111122223333:role/iam-role-name
```

## Trust relationships for the IAM role

Create the trust relationships for the IAM Role that will be used to authenticate and assume the user role in the k8s cluster.

1. Open the [IAM console](https://console.aws.amazon.com/iam/)
2. In the navigation pane, choose Roles
3. Choose the **role** that you want to check
4. Choose the Trust Relationships tab to verify that the format of the policy matches the format of the following JSON policy:

```json
{
  "Version": "2022--17",
  "Statement": [
    {
      "Effect": "Allow",
      "Principal": {
        "Federated": "arn:aws:iam::ACCOUNT_ID:oidc-provider/oidc.eks.AWS_REGION.amazonaws.com/id/EXAMPLED539D4633E53DE1B716D3041E"
      },
      "Action": "sts:AssumeRoleWithWebIdentity",
      "Condition": {
        "StringEquals": {
          "oidc.eks.AWS_REGION.amazonaws.com/id/EXAMPLED539D4633E53DE1B716D3041E:sub": "system:serviceaccount:SERVICE_ACCOUNT_NAMESPACE:SERVICE_ACCOUNT_NAME",
          "oidc.eks.AWS_REGION.amazonaws.com/id/EXAMPLED539D4633E53DE1B716D3041E:aud": "sts.amazonaws.com"
        }
      }
    }
  ]
}
```


