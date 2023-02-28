namespace Middleware.RedisInterface.Contracts.Responses;

public class GetAllCloudsResponse
{
    public IEnumerable<CloudResponse> Clouds { get; init; } = Enumerable.Empty<CloudResponse>();
}