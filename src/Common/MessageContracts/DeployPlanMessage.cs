using Middleware.Common.Models;

namespace Middleware.Common.MessageContracts;

public record DeployPlanMessage
{
    public TaskModel Task { get; init; }

    public Guid RobotId { get; init; }
}