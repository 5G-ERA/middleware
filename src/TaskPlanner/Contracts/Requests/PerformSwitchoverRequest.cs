namespace Middleware.TaskPlanner.Contracts.Requests;

public record PerformSwitchoverRequest
{
    public Guid ActionPlanId { get; init; }
    public Guid InstanceId { get; init; }
    public string Destination { get; init; }
    public string DestinationType { get; init; }
};