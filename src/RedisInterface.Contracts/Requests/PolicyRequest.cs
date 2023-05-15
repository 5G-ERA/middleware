namespace Middleware.RedisInterface.Contracts.Requests;

public class PolicyRequest
{
    public string Name { get; init; }
    public string Type { get; init; }
    public string Scope { get; init; }
    public bool IsActive { get; init; }
    public string Description { get; init; }
    public int IsExclusiveWithinType { get; init; }
    public DateTime LastTimeUpdated { get; init; }
}