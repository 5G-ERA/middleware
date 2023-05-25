namespace Middleware.RedisInterface.Contracts.Requests;

public class RegisterSlicesRequest
{
    public IEnumerable<SliceRequest> Slices { get; set; } = Enumerable.Empty<SliceRequest>();
    public LocationRequest Location { get; init; }
}

public class LocationRequest
{
    public Guid Id { get; init; }
    public string Type { get; init; }
    public string Name { get; init; }
    public string Organization { get; init; }
}

public class SliceRequest
{
    public string SliceId { get; init; }
    public string Site { get; init; }
    public int ExpDataRateUl { get; init; }
    public int ExpDataRateDl { get; init; }
    public int? Latency { get; init; }
    public int? Jitter { get; init; }
    public int? UserDensity { get; init; }
    public int? UserSpeed { get; init; }
    public string TrafficType { get; init; }
    public IEnumerable<string> Imsi { get; init; } = Enumerable.Empty<string>();
}