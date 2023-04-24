namespace Middleware.RedisInterface.Contracts.Responses;

public class GetInstancesResponse
{
    public IEnumerable<InstanceResponse> Instances { get; init; } = Enumerable.Empty<InstanceResponse>();
}