namespace Middleware.Orchestrator.Contracts.Requests;

public class CreateNetAppHeartbeatRequest
{
    public Guid Id { get; set; }
    public string? Name { get; set; }
    public int HardLimit { get; set; }
    public int OptimalLimit { get; set; }
    public int? CurrentRobotsCount { get; set; }
}