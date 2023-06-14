namespace Middleware.Common.MessageContracts;

public record ConnectRobotToSliceMessage : Message
{
    public Guid RobotId { get; init; }
    public string Location { get; init; }

    public string Imsi { get; init; }

    public string Slice { get; init; }
    public Guid ActionPlanId { get; init; }
}