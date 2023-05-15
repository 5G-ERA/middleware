namespace Middleware.RedisInterface.Contracts.Responses;

public class PolicyResponse
{
    public Guid Id { get; init; }
    public string Name { get; init; }
    public string Type { get; init; }
    public string Scope { get; init; }
    public bool IsActive { get; init; }
    public string Description { get; init; }
    public int IsExclusiveWithinType { get; init; }
    public DateTime LastTimeUpdated { get; init; }
    public string Priority { get; init; }
}