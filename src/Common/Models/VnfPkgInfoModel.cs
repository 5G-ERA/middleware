using System.Text.Json.Serialization;

namespace Middleware.Common.Models;

public class VnfPkgInfoModel
{
    [JsonPropertyName("id")]
    public Guid Id { get; set; }
    [JsonPropertyName("name")]
    public string Name { get; set; }
    [JsonPropertyName("description")]
    public string Description { get; set; }
}