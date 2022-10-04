using System.Text.Json.Serialization;

namespace Middleware.Common.Models;

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
    public int VirtualRam { get; set; }

    [JsonPropertyName("CPU")]
    public int Cpu { get; set; }

    [JsonPropertyName("RAM")]
    public int Ram { get; set; }

    [JsonPropertyName("MacAddress")]
    public string MacAddress { get; set; }

}