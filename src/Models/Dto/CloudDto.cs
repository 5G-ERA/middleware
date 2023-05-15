using Middleware.Models.Domain;
using Middleware.Models.Dto.Hardware;
using Redis.OM.Modeling;

namespace Middleware.Models.Dto;

[Document(IndexName = "cloud-idx", StorageType = StorageType.Json, Prefixes = new[] { CloudDto.Prefix })]
public class CloudDto : Dto
{
    public const string Prefix = "Cloud";

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
    public string? CloudStatus { get; set; }

    [Indexed]
    public Uri CloudIp { get; set; }

    [Indexed]
    public HardwareSpec HardwareSpec { get; set; }
    
    [Indexed]
    public string MacAddress { get; set; }

    [Indexed(Sortable = true)]
    public DateTimeOffset LastUpdatedTime { get; set; }

    [Indexed]
    public bool IsOnline { get; set; }

    public override BaseModel ToModel()
    {
        var dto = this;
        return new CloudModel()
        {
            Id = Guid.Parse(dto.Id!.Replace(Prefix, "")),
            Name = dto.Name,
            Organization = dto.Organization,
            CloudStatus = dto.CloudStatus,
            CloudIp = dto.CloudIp,
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