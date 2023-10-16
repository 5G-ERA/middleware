namespace Middleware.RedisInterface.Contracts.Responses;

public class LocationResponse
{
    public Guid Id { get; init; }
    public string Name { get; init; } = default!;
    public string Type { get; init; } = default!;
    public bool IsOnline { get; init; } = true;

    public Uri IpAddress { get; init; }
    public IEnumerable<SliceResponse> Slices { get; init; } = Enumerable.Empty<SliceResponse>();
}