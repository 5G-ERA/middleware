using Middleware.Models.Domain;

namespace Middleware.Common.MessageContracts;

public record RequestResourcePlanMessage : Message
{
    public TaskModel Task { get; set; }
    public RobotModel Robot { get; set; }
    public bool IsSuccess { get; set; }
    public string Error { get; set; }
}