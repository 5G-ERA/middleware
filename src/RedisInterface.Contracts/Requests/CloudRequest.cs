﻿namespace Middleware.RedisInterface.Contracts.Requests;

public class CloudRequest
{
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
}