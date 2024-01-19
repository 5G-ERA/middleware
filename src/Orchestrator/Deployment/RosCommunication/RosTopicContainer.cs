using System.Text.Json.Serialization;
using JetBrains.Annotations;
using Middleware.Models.Domain.Ros;

namespace Middleware.Orchestrator.Deployment.RosCommunication;

/// <summary>
///     Temporary class to parse the RosTopicModel into the correct format
/// </summary>
internal class RosTopicContainer
{
    [JsonPropertyName("topic_name")]
    public string Name { [UsedImplicitly] get; init; }

    [JsonPropertyName("topic_type")]
    public string Type { [UsedImplicitly] get; init; }

    public string Compression { [UsedImplicitly] get; set; } = "none";

    [JsonPropertyName("qos")]
    [CanBeNull]
    public Qos Qos { get; set; }

    public static RosTopicContainer FromRosTopicModel(RosTopicModel topic)
    {
        return new()
        {
            Name = topic.Name,
            Type = topic.Type
        };
    }
}