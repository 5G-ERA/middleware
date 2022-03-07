using System.Text.Json.Serialization;

namespace Middleware.Common.Models;

public class ContainerImageModel
{
    [JsonPropertyName("ImageId")]
    public string ImageId { get; set; }

    [JsonPropertyName("Name")]
    public string Name { get; set; }

    [JsonPropertyName("Timestamp")]
    public string Timestamp { get; set; }

    [JsonPropertyName("Description")]
    public string Description { get; set; }
}