using System.Text.Json.Serialization;
using JetBrains.Annotations;
using Middleware.Models.Domain.Ros;

namespace Middleware.Orchestrator.Deployment.RosCommunication.Structures;

internal class QosContainer
{
    [JsonPropertyName("preset")]
    public string Preset { get; set; } = default!;
    [JsonPropertyName("history")]
    public string History { get; set; } = default!;
    [JsonPropertyName("depth")]
    public int? Depth { get; set; } = default!;
    [JsonPropertyName("reliability")]
    public string Reliability { get; set; } = default!;
    [JsonPropertyName("durability")]
    public string Durability { get; set; } = default!;
    [JsonPropertyName("deadline")]
    public string Deadline { get; set; } = default!;
    [JsonPropertyName("lifespan")]
    public string Lifespan { get; set; } = default!;
    [JsonPropertyName("liveliness")]
    public string Liveliness { get; set; } = default!;
    [JsonPropertyName("lease")]
    public string Lease { get; set; } = default!;

    [CanBeNull]
    public static QosContainer FromQosModel(Qos x)
    {
        if (x is null)
        {
            return null;
        }
        return new()
        {
            Deadline = x.Deadline,
            Depth = x.Depth,
            Durability = x.Durability,
            History = x.History,
            Lease = x.Lease,
            Lifespan = x.Lifespan,
            Liveliness = x.Liveliness,
            Reliability = x.Reliability,
            Preset = x.Preset
        };
    }
}