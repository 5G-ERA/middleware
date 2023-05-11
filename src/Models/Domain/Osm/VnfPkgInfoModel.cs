using System.Text.Json.Serialization;

namespace Middleware.Models.Domain.Osm;

public class VnfPkgInfoModel
{
    [JsonPropertyName("id")]
    public Guid Id { get; set; }

    [JsonPropertyName("name")]
    public string Name { get; set; }

    [JsonPropertyName("description")]
    public string Description { get; set; }
}