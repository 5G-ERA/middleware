using Middleware.Models.Dto;
using System.Text.Json.Serialization;

namespace Middleware.Models.Domain;

public class CloudModel : BaseModel
{
    [JsonPropertyName("Id")]
    public override Guid Id { get; set; }

    [JsonPropertyName("Name")]
    public override string Name { get; set; }
    

    [JsonPropertyName("Type")]
    public string Type { get; set; }

    [JsonPropertyName("CloudStatus")]
    public string CloudStatus { get; set; }

    [JsonPropertyName("CloudIp")]
    public Uri CloudIp { get; set; }

    [JsonPropertyName("NumberOfCores")]
    public int NumberOfCores { get; set; }

    [JsonPropertyName("DiskStorage")]
    public long DiskStorage { get; set; }

    [JsonPropertyName("VirtualRam")]
    public long? VirtualRam { get; set; }

    [JsonPropertyName("CPU")]
    public int Cpu { get; set; }

    [JsonPropertyName("RAM")]
    public long Ram { get; set; }

    [JsonPropertyName("MacAddress")]
    public string MacAddress { get; set; }

    [JsonPropertyName("LastUpdatedTime")]
    public DateTime LastUpdatedTime { get; set; }

    [JsonPropertyName("IsOnline")]
    public bool IsOnline { get; set; }

    /// <summary>
    /// Onboarding validation of the cloud data object.
    /// </summary>
    /// <returns></returns>
    public bool IsValid()
    {
        if (string.IsNullOrEmpty(Name.ToString())) return false;
        if (string.IsNullOrEmpty(CloudIp.ToString())) return false;
        if (string.IsNullOrEmpty(NumberOfCores.ToString())) return false;
        if (string.IsNullOrEmpty(DiskStorage.ToString())) return false;
        //if (string.IsNullOrEmpty(MacAddress.ToString())) return false;
        if (string.IsNullOrEmpty(Ram.ToString())) return false;
        return true;
    }
    public override Dto.Dto ToDto()
    {
        var domain = this;
        return new CloudDto()
        {
            Id = domain.Id.ToString(),
            Name = domain.Name,
            Type = domain.Type,
            CloudStatus = domain.CloudStatus,
            CloudIp = domain.CloudIp,
            HardwareSpec = new()
            {
                Cpu = domain.Cpu,
                StorageDisk = domain.DiskStorage,
                VirtualRam = domain.VirtualRam,
                Ram = domain.Ram,
                NumberCores = domain.NumberOfCores
            },
            MacAddress = domain.MacAddress,
            LastUpdatedTime = domain.LastUpdatedTime,
            IsOnline = domain.IsOnline
        };
    }
}