using Middleware.Models.Domain;

namespace Middleware.Common.MessageContracts;

public record RequestResourcePlanMessage : Message
{
    public TaskModel Task { get; set; }
    public RobotModel Robot { get; set; }
}