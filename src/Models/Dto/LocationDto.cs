using Middleware.Models.Domain;
using Middleware.Models.Dto.Hardware;
using Middleware.Models.Enums;
using Redis.OM.Modeling;

namespace Middleware.Models.Dto;

[Document(IndexName = "location-idx", StorageType = StorageType.Json, Prefixes = new[] { Prefix })]
public class LocationDto : Dto
{
    public const string Prefix = "Location";

    [Indexed]
    [RedisIdField]
    public override string Id { get; set; } = default!;

    [Indexed]
    public string? Name { get; set; }

    [Indexed]
    public string Type { get; set; } = default!;

    [Indexed]
    public string Organization { get; set; } = default!;

    [Indexed]
    public string? Status { get; set; }

    [Indexed]
    public Uri Address { get; set; } = default!;

    [Indexed]
    public HardwareSpec? HardwareSpec { get; set; }

    [Indexed]
    public string? MacAddress { get; set; }

    [Indexed(Sortable = true)]
    public DateTimeOffset LastUpdatedTime { get; set; }

    [Indexed]
    public bool IsOnline { get; set; }

    [Indexed]
    public int? Throughput { get; set; }

    [Indexed]
    public int? Latency { get; set; }


    /// <inheritdoc />
    public override BaseModel ToModel()
    {
        var dto = this;
        return new Location
        {
            Id = Guid.Parse(dto.Id!.Replace(Prefix, "")),
            Name = dto.Name!,
            Organization = dto.Organization,
            Status = dto.Status!,
            Address = dto.Address,
            NumberOfCores = dto.HardwareSpec?.NumberCores,
            DiskStorage = dto.HardwareSpec?.StorageDisk,
            VirtualRam = dto.HardwareSpec?.VirtualRam,
            Cpu = dto.HardwareSpec?.Cpu,
            Ram = dto.HardwareSpec?.Ram,
            MacAddress = dto.MacAddress,
            LastUpdatedTime = dto.LastUpdatedTime.DateTime,
            IsOnline = dto.IsOnline,
            Type = Enum.Parse<LocationType>(dto.Type),
            Latency = dto.Latency,
            Throughput = dto.Throughput
        };
    }
}