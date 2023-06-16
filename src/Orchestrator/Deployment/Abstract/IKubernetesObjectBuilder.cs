using k8s.Models;
using Middleware.Common.Enums;

namespace Middleware.Orchestrator.Deployment;

internal interface IKubernetesObjectBuilder
{
    V1Deployment CreateStartupDeployment(string name, string tag);

    V1Deployment SerializeAndConfigureDeployment(string deployment, Guid serviceInstanceId);
    V1Service SerializeAndConfigureService(string service, string name, Guid serviceInstanceId);

    V1Service CreateDefaultService(string deploymentName, Guid serviceInstanceId);
    V1Service CreateStartupService(string serviceImageName, K8SServiceKindEnum kind, V1ObjectMeta meta);
}