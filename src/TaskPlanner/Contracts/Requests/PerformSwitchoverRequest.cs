namespace Middleware.TaskPlanner.Contracts.Requests;

public record PerformSwitchoverRequest
{
    public Guid ActionPlanId { get; init; }
    public Guid ActionId { get; init; }
    public string Destination { get; init; }
    public string DestinationType { get; init; }
};