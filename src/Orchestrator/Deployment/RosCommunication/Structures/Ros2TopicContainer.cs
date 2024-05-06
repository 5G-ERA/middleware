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
    [JsonIgnore(Condition = JsonIgnoreCondition.WhenWritingNull)]
    [CanBeNull]
    public string Compression { [UsedImplicitly] get; set; }

    [JsonPropertyName("qos")]
    [JsonIgnore(Condition = JsonIgnoreCondition.WhenWritingNull)]
    [CanBeNull]
    public QosContainer Qos { [UsedImplicitly]get; set; }

    public static Ros2TopicContainer FromRosTopicModel(RosTopicModel topic)
    {
        return new()
        {
            Name = topic.Name,
            Type = topic.Type,
            Compression = topic.Compression,
            Qos = QosContainer.FromQosModel(topic.Qos)
        };
    }
}