using System.Text.Json.Serialization;
using Middleware.Models.Domain.Ros;

namespace Middleware.Orchestrator.Deployment.RosCommunication.Structures;

/// <summary>
///     Represents the structure
/// </summary>
internal class RosTransformsContainer
{
    [JsonPropertyName("source_frame")]
    public string SourceFrame { get; set; }

    [JsonPropertyName("target_frame")]
    public string TargetFrame { get; set; }

    [JsonPropertyName("angular_thres")]
    public double AngularThres { get; set; }

    [JsonPropertyName("trans_thres")]
    public double TransThres { get; set; }

    [JsonPropertyName("max_publish_period")]
    public double MaxPublishPeriod { get; set; }

    public static RosTransformsContainer FromRosTransformsModel(RosTransformsModel x)
    {
        return new()
        {
            AngularThres = x.AngularThres,
            SourceFrame = x.SourceFrame,
            TargetFrame = x.TargetFrame,
            TransThres = x.TransThres,
            MaxPublishPeriod = x.MaxPublishPeriod
        };
    }
}