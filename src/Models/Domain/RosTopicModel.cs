using System.Text.Json.Serialization;
using Middleware.Models.Dto.Ros;

namespace Middleware.Models.Domain;

public class RosTopicModel
{
    [JsonPropertyName("topic_name")]
    //[JsonConverter(typeof(TopicNameConverter))]
    public string Name { get; set; } = default!;

    [JsonPropertyName("topic_type")]
    public string? Type { get; set; }

    [JsonIgnore]
    public string? Description { get; set; }

    [JsonIgnore]
    public bool Enabled { get; set; }

    /// <summary>
    ///     Set the topic to be enabled.
    /// </summary>
    public void Enable()
    {
        Enabled = true;
    }

    /// <summary>
    ///     Set the topic to be disabled.
    /// </summary>
    public void Disable()
    {
        Enabled = false;
    }

    public RosTopic ToDto()
    {
        return new()
        {
            Name = Name,
            Type = Type,
            Description = Description,
            Enabled = Enabled
        };
    }
}