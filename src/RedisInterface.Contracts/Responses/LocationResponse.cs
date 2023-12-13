namespace Middleware.RedisInterface.Contracts.Responses;

public class LocationResponse
{
    public Guid Id { get; init; }
    public string Name { get; init; } = default!;
    public string Type { get; init; } = default!;
    public bool IsOnline { get; init; }

    public string Organization { get; init; } = default!;
    public Uri IpAddress { get; init; }
    public string Status { get; init; }
    public string MacAddress { get; init; }
    public int? Cpu { get; init; }
    public int? NumberOfCores { get; init; }
    public long? Ram { get; init; }
    public long? VirtualRam { get; init; }
    public long? DiskStorage { get; init; }
    public DateTime LastUpdatedTime { get; init; }
    public int? Throughput { get; init; }
    public int? Latency { get; init; }
    public IEnumerable<SliceResponse> Slices { get; init; } = Enumerable.Empty<SliceResponse>();
}

public class GetLocationsResponse
{
    public IEnumerable<LocationResponse> Locations { get; init; } = Enumerable.Empty<LocationResponse>();
}