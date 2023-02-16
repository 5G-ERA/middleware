using Middleware.Common.Models;

namespace Middleware.Common.MessageContracts;

public record DeployPlanMessage : Message
{
    /// <summary>
    /// Middleware location name that 
    /// </summary>
    public string DeploymentLocation { get; init; }
    /// <summary>
    /// The constructed task to be deployed
    /// </summary>
    public TaskModel Task { get; init; }
    /// <summary>
    /// Robot identifier
    /// </summary>
    public Guid RobotId { get; init; }
}