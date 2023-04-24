namespace Middleware.RedisInterface.Contracts.Responses;

public class GetCloudsResponse
{
    public IEnumerable<CloudResponse> Clouds { get; init; } = Enumerable.Empty<CloudResponse>();
}