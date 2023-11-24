using Middleware.Models.Domain.Contracts;
using Middleware.Models.Dto;
using Middleware.Models.Enums;

namespace Middleware.Models.Domain;

public class Location : BaseModel, ILocation
{
    public string Status { get; set; } = default!;
    public int? NumberOfCores { get; set; }
    public long? DiskStorage { get; set; }
    public long? VirtualRam { get; set; }
    public int? Cpu { get; set; }
    public long? Ram { get; set; }
    public string? MacAddress { get; set; }
    public DateTime LastUpdatedTime { get; set; }
    public bool IsOnline { get; set; }
    public override Guid Id { get; set; } = Guid.NewGuid();
    public LocationType Type { get; set; }

    public override string Name { get; set; } = default!;

    public Uri Address { get; set; } = default!;
    public string Organization { get; set; } = default!;

    /// <inheritdoc />
    public BaseModel ToBaseLocation()
    {
        return this;
    }

    /// <inheritdoc />
    public Location ToLocation()
    {
        return this;
    }

    /// <inheritdoc />
    public string GetNetAppAddress(string netAppName)
    {
        return GetNetAppAddress(netAppName, Address);
    }

    /// <inheritdoc />
    public string GetNetAppStatusReportAddress()
    {
        return GetNetAppReportAddress(Address);
    }

    /// <inheritdoc />
    public override Dto.Dto ToDto()
    {
        var domain = this;
        return new LocationDto
        {
            Id = domain.Id.ToString(),
            Name = domain.Name,
            Type = Type.ToString(),
            Organization = domain.Organization,
            Status = domain.Status,
            Address = domain.Address,
            HardwareSpec = new()
            {
                Cpu = domain.Cpu,
                StorageDisk = domain.DiskStorage,
                VirtualRam = domain.VirtualRam,
                Ram = domain.Ram,
                NumberCores = domain.NumberOfCores
            },
            MacAddress = domain.MacAddress,
            LastUpdatedTime = domain.LastUpdatedTime == default ? DateTimeOffset.Now : domain.LastUpdatedTime,
            IsOnline = domain.IsOnline
        };
    }
}