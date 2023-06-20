using k8s.Models;
using Middleware.Models.Domain;

namespace Middleware.Orchestrator.Models;

public record DeploymentPair(V1Deployment Deployment, V1Service Service, Guid InstanceId, InstanceModel Instance)
{
    /// <summary>
    ///     Deployed deployment
    /// </summary>
    public V1Deployment Deployment { get; init; } = Deployment;

    /// <summary>
    ///     Deployed service
    /// </summary>
    public V1Service Service { get; init; } = Service;

    /// <summary>
    ///     Identifier of the deployed instance
    /// </summary>
    public Guid InstanceId { get; init; } = InstanceId;

    /// <summary>
    ///     Instance to be deployed
    /// </summary>
    public InstanceModel Instance { get; init; } = Instance;

    /// <summary>
    ///     Name of the Network Application to be deployed
    /// </summary>
    public string Name { get; init; } = Instance.Name;
}