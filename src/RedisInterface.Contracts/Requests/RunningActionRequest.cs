namespace Middleware.RedisInterface.Contracts.Requests;

public class RunningActionRequest
{
    public Guid Id { get; set; }
    
    public Guid ActionId { get; set; } 
    
    public Guid ActionPlanId { get; set; }
    
    public string Name { get; set; } = default!;

    public List<string>? Tags { get; set; }

    public int Order { get; set; }

    public string Placement { get; set; } = default!;

    public string PlacementType { get; set; } 

    public string ActionPriority { get; set; }

    public string ActionStatus { get; set; }

    public IEnumerable<InstanceRunningRequest> Services { get; set; }
}