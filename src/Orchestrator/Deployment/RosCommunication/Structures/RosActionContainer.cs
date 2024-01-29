using System.Text.Json.Serialization;
using Middleware.Models.Domain.Ros;

namespace Middleware.Orchestrator.Deployment.RosCommunication.Structures;

internal class RosActionContainer
{
    [JsonPropertyName("name")]
    public string Name { get; set; } = default!;

    [JsonPropertyName("type")]
    public string Type { get; set; } = default!;

    public static RosActionContainer FromRosActionModel(RosActionModel x)
    {
        return new()
        {
            Name = x.Name,
            Type = x.Type
        };
    }
}