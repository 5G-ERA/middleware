using System.Text.Json.Serialization;
using JetBrains.Annotations;
using Middleware.Models.Domain;

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

    public static RosTopicContainer FromRosTopicModel(RosTopicModel topic)
    {
        return new()
        {
            Name = topic.Name,
            Type = topic.Type
        };
    }
}