using k8s.Models;
using Middleware.Common.Enums;
using Middleware.Orchestrator.Models;

namespace Middleware.Orchestrator.Deployment;

internal interface IKubernetesObjectBuilder
{
    V1Deployment CreateStartupDeployment(string name, string tag);

    V1Deployment SerializeAndConfigureDeployment(string deployment, Guid serviceInstanceId, string name);
    V1Service SerializeAndConfigureService(string service, string name, Guid serviceInstanceId);

    V1Service CreateDefaultService(string deploymentName, Guid serviceInstanceId, V1Deployment depl);
    V1Service CreateStartupService(string serviceImageName, K8SServiceKind kind, V1ObjectMeta meta);
    void ConfigureCrossNetAppConnection(IReadOnlyList<DeploymentPair> netApps);
}