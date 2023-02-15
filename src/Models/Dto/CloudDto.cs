﻿using Middleware.Models.Domain;
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
    public string? CloudStatus { get; set; }
    [Indexed]
    public Uri CloudIp { get; set; }
    [Indexed(Sortable = true)]
    public int NumberOfCores { get; set; }
    [Indexed(Sortable = true)]
    public long DiskStorage { get; set; }
    [Indexed(Sortable = true)]
    public int VirtualRam { get; set; }
    [Indexed(Sortable = true)]
    public int Cpu { get; set; }
    [Indexed(Sortable = true)]
    public int Ram { get; set; }
    [Indexed]
    public string MacAddress { get; set; }
    [Indexed(Sortable = true)]
    public DateTime LastUpdatedTime { get; set; }
    [Indexed]
    public bool IsOnline { get; set; }

    public override BaseModel ToModel()
    {
        var dto = this;
        return new CloudModel()
        {
            Id = Guid.Parse(dto.Id!.Replace(Prefix, "")),
            Name = dto.Name,
            Type = dto.Type,
            CloudStatus = dto.CloudStatus,
            CloudIp = dto.CloudIp,
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