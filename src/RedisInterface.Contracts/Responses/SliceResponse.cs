namespace Middleware.RedisInterface.Contracts.Responses;

public class SliceResponse
{
    public Guid Id { get; init; }
    public string Name { get; init; }
    public string Site { get; init; }
    public string SliceType { get; init; }
    public int ExpDataRateUl { get; init; }
    public int ExpDataRateDl { get; init; }
    public int? Latency { get; init; }
    public int? Jitter { get; init; }
    public int? UserDensity { get; init; }
    public int? UserSpeed { get; init; }
    public string TrafficType { get; init; }
    public IEnumerable<string> Imsi { get; init; } = Enumerable.Empty<string>();
}