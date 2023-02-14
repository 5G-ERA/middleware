using Middleware.Common.Models;

namespace Middleware.Common.MessageContracts;

public record DeployPlanMessage
{
    public string Message { get; init; }
    public string DeploymentLocation { get; init; }
    // public TaskModel Task { get; init; }
    //
    // public Guid RobotId { get; init; }
}