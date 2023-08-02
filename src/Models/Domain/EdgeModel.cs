using System.Text.Json.Serialization;
using Middleware.Models.Domain.Contracts;
using Middleware.Models.Dto;
using Middleware.Models.Enums;

namespace Middleware.Models.Domain;

public class EdgeModel : BaseModel, ILocation
{
    [JsonPropertyName("EdgeStatus")]
    public string EdgeStatus { get; set; } = default!;

    [JsonPropertyName("EdgeIp")]
    public Uri EdgeIp { get; set; } = default!;

    [JsonPropertyName("MacAddress")]
    public string? MacAddress { get; set; }

    [JsonPropertyName("CPU")]
    public int? Cpu { get; set; }

    [JsonPropertyName("RAM")]
    public long? Ram { get; set; }

    [JsonPropertyName("VirtualRam")]
    public long? VirtualRam { get; set; }

    [JsonPropertyName("DiskStorage")]
    public long? DiskStorage { get; set; }

    [JsonPropertyName("NumberOfCores")]
    public int? NumberOfCores { get; set; }

    [JsonPropertyName("LastUpdatedTime")]
    public DateTime LastUpdatedTime { get; set; }

    [JsonPropertyName("IsOnline")]
    public bool IsOnline { get; set; }

    /// <inheritdoc />
    public string? ApiKey { get; set; }

    [JsonPropertyName("Id")]
    public override Guid Id { get; set; } = Guid.NewGuid();

    /// <inheritdoc />
    public Uri Address => EdgeIp;

    [JsonPropertyName("Name")]
    public override string Name { get; set; } = default!;

    [Obsolete]
    [JsonPropertyName("Type")]
    public LocationType Type { get; set; } = LocationType.Edge;

    [JsonPropertyName("Organization")]
    public string Organization { get; set; } = default!;

    /// <inheritdoc />
    public BaseModel ToBaseLocation()
    {
        return this;
    }

    /// <inheritdoc />
    public Location ToLocation()
    {
        var that = this;
        return new()
        {
            Name = that.Name,
            Organization = that.Organization,
            Type = LocationType.Edge,
            Address = that.EdgeIp,
            Id = that.Id
        };
    }

    /// <inheritdoc />
    public string GetNetAppAddress(string netAppName)
    {
        return GetNetAppAddress(netAppName, EdgeIp);
    }

    /// <inheritdoc />
    public string GetNetAppStatusReportAddress()
    {
        return GetNetAppReportAddress(EdgeIp);
    }

    /// <summary>
    ///     Onboarding validation of the edge data object.
    /// </summary>
    /// <returns></returns>
    public bool IsValid()
    {
        if (string.IsNullOrEmpty(Name)) return false;
        if (string.IsNullOrEmpty(EdgeIp.ToString())) return false;
        if (string.IsNullOrEmpty(NumberOfCores.ToString())) return false;
        if (string.IsNullOrEmpty(DiskStorage.ToString())) return false;
        // if (string.IsNullOrEmpty(MacAddress.ToString())) return false;
        if (string.IsNullOrEmpty(Ram.ToString())) return false;
        if (string.IsNullOrEmpty(Organization)) return false;
        return true;
    }

    public override Dto.Dto ToDto()
    {
        var domain = this;
        return new EdgeDto
        {
            Id = domain.Id.ToString(),
            Name = domain.Name,
            Type = LocationType.Edge.ToString(),
            Organization = domain.Organization,
            EdgeStatus = domain.EdgeStatus,
            EdgeIp = domain.EdgeIp,
            MacAddress = domain.MacAddress,
            HardwareSpec = new()
            {
                Cpu = domain.Cpu,
                Ram = domain.Ram,
                NumberCores = domain.NumberOfCores,
                StorageDisk = domain.DiskStorage,
                VirtualRam = domain.VirtualRam
            },
            LastUpdatedTime = domain.LastUpdatedTime == default ? DateTimeOffset.Now : domain.LastUpdatedTime,
            IsOnline = domain.IsOnline
        };
    }
}