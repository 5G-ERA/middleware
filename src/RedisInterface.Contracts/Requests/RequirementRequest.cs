namespace Middleware.RedisInterface.Contracts.Requests;

public class RequirementRequest
{
    public long Minimum { get; init; }
    public long Optimal { get; init; }
    public string Priority { get; init; }
}