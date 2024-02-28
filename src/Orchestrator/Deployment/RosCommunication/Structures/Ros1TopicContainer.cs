using System.Text.Json.Serialization;
using JetBrains.Annotations;
using Middleware.Models.Domain.Ros;

namespace Middleware.Orchestrator.Deployment.RosCommunication.Structures;

/// <summary>
///     Temporary class to parse the RosTopicModel into the correct format
/// </summary>
internal class Ros1TopicContainer
{
    [JsonPropertyName("topic_name")]
    public string Name { [UsedImplicitly] get; init; }

    [JsonPropertyName("topic_type")]
    public string Type { [UsedImplicitly] get; init; }

    [JsonPropertyName("compression")]
    public string Compression { [UsedImplicitly] get; set; } = "none";

    [JsonPropertyName("qos")]
    [CanBeNull]
    public Qos Qos { [UsedImplicitly]get; set; }

    public static Ros1TopicContainer FromRosTopicModel(RosTopicModel topic)
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