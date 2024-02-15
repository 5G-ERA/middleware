namespace Middleware.Orchestrator.Contracts.Responses;

public class GetNetAppHeartbeatResponse
{
    public Guid Id { get; set; }
    
    public string? Name { get; set; } = default!;

    public int HardLimit { get; set; }

    public int OptimalLimit { get; set; }

    public int? CurrentRobotsCount { get; set; }

    public DateTimeOffset Timestamp { get; set; }

    public string? Colour { get; set; }
}