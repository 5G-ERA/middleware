namespace Middleware.RedisInterface.Contracts.Responses;

public class GetAllInstancesResponse
{
    public IEnumerable<InstanceResponse> Instances { get; init; } = Enumerable.Empty<InstanceResponse>();
}