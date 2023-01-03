using System.Text.Json.Serialization;

namespace Middleware.Common.Models;

public class EdgeModel : BaseModel
{
    [JsonPropertyName("Id")]
    public override Guid Id { get; set; }

    [JsonPropertyName("Name")]
    public override string Name { get; set; }

    [JsonPropertyName("Type")]
    public string Type { get; set; }

    [JsonPropertyName("EdgeStatus")]
    public string EdgeStatus { get; set; }

    [JsonPropertyName("EdgeIp")]
    public Uri EdgeIp { get; set; }

    [JsonPropertyName("MacAddress")]
    public string MacAddress { get; set; }

    [JsonPropertyName("CPU")]
    public int Cpu { get; set; }

    [JsonPropertyName("RAM")]
    public int Ram { get; set; }

    [JsonPropertyName("VirtualRam")]
    public int VirtualRam { get; set; }

    [JsonPropertyName("DiskStorage")]
    public long DiskStorage { get; set; }

    [JsonPropertyName("NumberOfCores")]
    public int NumberOfCores { get; set; }

    [JsonPropertyName("LastUpdatedTime")]
    public DateTime LastUpdatedTime { get; set; }

    [JsonPropertyName("IsOnline")]
    public bool IsOnline { get; set; }

    /// <summary>
    /// Onboarding validation of the edge data object.
    /// </summary>
    /// <returns></returns>
    public bool IsValid()
    {
        if (string.IsNullOrEmpty(Name.ToString())) return false;
        if (string.IsNullOrEmpty(EdgeIp.ToString())) return false;
        if (string.IsNullOrEmpty(NumberOfCores.ToString())) return false;
        if (string.IsNullOrEmpty(DiskStorage.ToString())) return false;
        // if (string.IsNullOrEmpty(MacAddress.ToString())) return false;
        if (string.IsNullOrEmpty(Ram.ToString())) return false;
        return true;
    }
}
