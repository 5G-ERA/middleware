using Middleware.Models.Domain;
using Redis.OM.Modeling;

namespace Middleware.Models.Dto;

[Document(IndexName = "edge-idx", StorageType = StorageType.Json, Prefixes = new[] { "Edge" })]
public class EdgeDto : Dto
{
    [Indexed]
    [RedisIdField]
    public override string Id { get; set; }

    [Indexed]
    public string? Name { get; set; }
    [Indexed]
    public string Type { get; set; }
    [Indexed]
    public string? EdgeStatus { get; set; }
    [Indexed]
    public Uri EdgeIp { get; set; }
    [Indexed]
    public string MacAddress { get; set; }
    [Indexed(Sortable = true)]
    public int Cpu { get; set; }
    [Indexed(Sortable = true)]
    public int Ram { get; set; }
    [Indexed(Sortable = true)]
    public int VirtualRam { get; set; }
    [Indexed(Sortable = true)]
    public long DiskStorage { get; set; }
    [Indexed(Sortable = true)]
    public int NumberOfCores { get; set; }
    [Indexed(Sortable = true)]
    public DateTime LastUpdatedTime { get; set; }
    [Indexed]
    public bool IsOnline { get; set; }

    public override BaseModel ToModel()
    {
        var dto = this;
        return new CloudModel()
        {
            Id = Guid.Parse(dto.Id!),
            Name = dto.Name,
            Type = dto.Type,
            CloudStatus = dto.EdgeStatus,
            CloudIp = dto.EdgeIp,
            NumberOfCores = dto.NumberOfCores,
            DiskStorage = dto.DiskStorage,
            VirtualRam = dto.VirtualRam,
            Cpu = dto.Cpu,
            Ram = dto.Ram,
            MacAddress = dto.MacAddress,
            LastUpdatedTime = dto.LastUpdatedTime,
            IsOnline = dto.IsOnline
        };
    }

}
