namespace Middleware.RedisInterface.Contracts.Responses;

public class GetEdgesResponse
{
    public IEnumerable<EdgeResponse> Edges { get; set; } = Enumerable.Empty<EdgeResponse>();
}