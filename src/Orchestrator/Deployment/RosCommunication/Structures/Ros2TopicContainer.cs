using System.Text.Json.Serialization;
using JetBrains.Annotations;
using Middleware.Models.Domain.Ros;

namespace Middleware.Orchestrator.Deployment.RosCommunication.Structures;

internal class Ros2TopicContainer
{
    [JsonPropertyName("name")]
    public string Name { [UsedImplicitly] get; init; }

    [JsonPropertyName("type")]
    public string Type { [UsedImplicitly] get; init; }

    [JsonPropertyName("compression")]
    public string Compression { [UsedImplicitly] get; set; } = "none";

    [JsonPropertyName("qos")]
    [CanBeNull]
    public Qos Qos { get; set; }

    public static Ros2TopicContainer FromRosTopicModel(RosTopicModel topic)
    {
        return new()
        {
            Name = topic.Name,
            Type = topic.Type,
            Compression = topic.Compression,
            Qos = topic.Qos
        };
    }
}