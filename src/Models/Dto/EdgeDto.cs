using Middleware.Models.Domain;
using Middleware.Models.Dto.Hardware;
using Middleware.Models.Enums;
using Redis.OM.Modeling;

namespace Middleware.Models.Dto;

[Document(IndexName = "edge-idx", StorageType = StorageType.Json, Prefixes = new[] { EdgeDto.Prefix })]
public class EdgeDto : Dto
{
    public const string Prefix = "Edge";
    [Indexed]
    [RedisIdField]
    public override string Id { get; set; }

    [Indexed]
    public string? Name { get; set; }
    [Indexed]
    public string Type { get; set; }

    [Indexed]
    public string Organization { get; set; }

    [Indexed]
    public string? EdgeStatus { get; set; }
    [Indexed]
    public Uri EdgeIp { get; set; }
    [Indexed]
    public string MacAddress { get; set; }

    [Indexed()]
    public HardwareSpec HardwareSpec { get; set; } = new();
    
    [Indexed(Sortable = true)]
    public DateTimeOffset LastUpdatedTime { get; set; } = DateTimeOffset.Now;
    [Indexed]
    public bool IsOnline { get; set; }

    public override BaseModel ToModel()
    {
        var dto = this;
        return new EdgeModel()
        {
            Id = Guid.Parse(dto.Id!.Replace(Prefix, "")),
            Name = dto.Name,
            Organization = dto.Organization,
            EdgeStatus = dto.EdgeStatus,
            EdgeIp = dto.EdgeIp,
            NumberOfCores = dto.HardwareSpec.NumberCores,
            DiskStorage = dto.HardwareSpec.StorageDisk,
            VirtualRam = dto.HardwareSpec.VirtualRam,
            Cpu = dto.HardwareSpec.Cpu,
            Ram = dto.HardwareSpec.Ram,
            MacAddress = dto.MacAddress,
            LastUpdatedTime = dto.LastUpdatedTime.DateTime,
            IsOnline = dto.IsOnline
        };
    }
}
