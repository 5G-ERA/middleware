﻿namespace Middleware.RedisInterface.Contracts.Responses;

public class CloudResponse
{
    public Guid Id { get; init; }
    public string Name { get; init; }
    public string Organization { get; set; }
    public string Type { get; init; }
    public string Status { get; init; }
    public Uri IpAddress { get; init; }
    public int? Cpu { get; init; }
    public int? NumberOfCores { get; init; }
    public long? DiskStorage { get; init; }
    public long? Ram { get; init; }
    public long? VirtualRam { get; init; }
    public string MacAddress { get; init; }
    public DateTime LastUpdatedTime { get; init; }
    public int? Throughput { get; init; }
    public int? Latency { get; init; }
}