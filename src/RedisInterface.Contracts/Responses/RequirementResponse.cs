namespace Middleware.RedisInterface.Contracts.Responses;

public class RequirementResponse
{
    public long Minimum { get; init; }
    public long Optimal { get; init; }
    public string Priority { get; init; }

    public RequirementResponse()
    {
    }

    public RequirementResponse(long minimum, long optimal, string priority)
    {
        Minimum = minimum;
        Optimal = optimal;
        Priority = priority;
    }
}