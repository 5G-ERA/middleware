using Redis.OM.Modeling;

namespace Middleware.Models.Dto.Hardware;

[Document(StorageType = StorageType.Json)]
public class HardwareRequirements
{
    [Indexed]
    public long? MinimumRam { get; set; }

    [Indexed]
    public long? OptimalRam { get; set; }

    [Indexed]
    public string? RamPriority { get; set; } = default!;

    [Indexed]
    public long? MinimumNumberOfCores { get; set; }

    [Indexed]
    public long? OptimalNumberOfCores { get; set; }

    [Indexed]
    public string? NumberOfCoresPriority { get; set; } = default!;

    [Indexed]
    public long? MinimumDiskStorage { get; set; }

    [Indexed]
    public long? OptimalDiskStorage { get; set; }

    [Indexed]
    public string? DiskStoragePriority { get; set; } = default!;

    [Indexed]
    public long? MinimumThroughput { get; set; }

    [Indexed]
    public long? OptimalThroughput { get; set; }

    [Indexed]
    public string? ThroughputPriority { get; set; } = default!;

    [Indexed]
    public long? MinimumLatency { get; set; }

    [Indexed]
    public long? OptimalLatency { get; set; }

    [Indexed]
    public string? LatencyPriority { get; set; } = default!;
}