using System.Text.Json.Serialization;

namespace Middleware.Models.Domain.Ros;

public class RosActionsModel
{
    [JsonPropertyName("name")]
    public string Name { get; set; } = default!;

    [JsonPropertyName("type")]
    public string Type { get; set; } = default!;
}