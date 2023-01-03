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

    public DeploymentPairModel(V1Deployment deployment, V1Service service)
    {
        Deployment = deployment;
        Service = service;
    }
}
