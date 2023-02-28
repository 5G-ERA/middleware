namespace Middleware.RedisInterface.Contracts.Responses;

public class GetAllEdgesResponse
{
    public IEnumerable<EdgeResponse> Edges { get; set; } = Enumerable.Empty<EdgeResponse>();
}