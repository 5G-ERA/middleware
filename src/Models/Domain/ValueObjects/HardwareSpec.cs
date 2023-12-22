namespace Middleware.Models.Domain.ValueObjects;

public class HardwareSpec
{
    public int? Cpu { get; set; }


    public long? Ram { get; set; }


    public long? StorageDisk { get; set; }


    public int? NumberCores { get; set; }


    public long? VirtualRam { get; set; }


    public long? Latency { get; set; }


    public long? Throughput { get; set; }
}