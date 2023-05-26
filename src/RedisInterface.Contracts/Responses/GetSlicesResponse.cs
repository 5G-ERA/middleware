namespace Middleware.RedisInterface.Contracts.Responses;

public class GetSlicesResponse
{
    public IEnumerable<SliceResponse> Slices { get; init; } = Enumerable.Empty<SliceResponse>();
}