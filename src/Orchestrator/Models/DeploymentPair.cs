using JetBrains.Annotations;
using k8s.Models;
using Middleware.Models.Domain;

namespace Middleware.Orchestrator.Models;

public record DeploymentPair()
{
    /// <summary>
    ///     Deployed deployment
    /// </summary>
    public V1Deployment Deployment { get; init; }

    /// <summary>
    ///     Deployed service
    /// </summary>
    public V1Service Service { get; init; }

    /// <summary>
    ///     Identifier of the deployed instance
    /// </summary>
    public Guid InstanceId { get; init; }

    /// <summary>
    ///     instance to be deployed
    /// </summary>
    [CanBeNull]
    public InstanceModel Instance { get; init; }

    /// <summary>
    ///     Name of the Network Application to be deployed
    /// </summary>
    public string Name { get; init; }

    public DeploymentPair(V1Deployment deployment, V1Service service, Guid instanceId,
        [NotNull] InstanceModel instance) : this()
    {
        Deployment = deployment;
        Service = service;
        InstanceId = instanceId;
        Instance = instance;
        Name = instance.Name;
    }

    public DeploymentPair(string name, V1Deployment deployment, V1Service service, Guid instanceId) : this()
    {
        Name = name;
        Deployment = deployment;
        Service = service;
        InstanceId = instanceId;
    }
}