using k8s.Models;

namespace Middleware.Orchestrator.Models;

public class DeploymentPairModel
{
    /// <summary>
    /// Deployed deployment
    /// </summary>
    public V1Deployment Deployment { get; set; }
    /// <summary>
    /// Deployed service
    /// </summary>
    public V1Service Service { get; set; }
    /// <summary>
    /// Identifier of the deployed instance
    /// </summary>
    public Guid InstanceId { get; set; }

    public DeploymentPairModel(V1Deployment deployment, V1Service service, Guid instanceId)
    {
        Deployment = deployment;
        Service = service;
        InstanceId = instanceId;
    }
}