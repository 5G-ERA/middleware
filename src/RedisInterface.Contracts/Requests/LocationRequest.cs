namespace Middleware.RedisInterface.Contracts.Requests;

public class LocationRequest
{
    public string Type { get; init; }
    public string Name { get; init; }
    public string Organization { get; init; }
    public string Status { get; init; }
    public Uri IpAddress { get; init; }
    public string MacAddress { get; init; }
    public int? Cpu { get; init; }
    public int? NumberOfCores { get; init; }
    public long? Ram { get; init; }
    public long? VirtualRam { get; init; }
    public long? DiskStorage { get; init; }
    public int? Throughput { get; init; }
    public int? Latency { get; init; }
}