using System.Text.Json.Serialization;

namespace Middleware.Models.Domain.Ros;

public class RosTransformsModel
{
    [JsonPropertyName("source_frame")]
    public string SourceFrame { get; set; } = default!;

    [JsonPropertyName("target_frame")]
    public string TargetFrame { get; set; } = default!;

    [JsonPropertyName("angular_thres")]
    public double AngularThres { get; set; }

    [JsonPropertyName("trans_thres")]
    public double TransThres { get; set; }

    [JsonPropertyName("max_publish_period")]
    public double MaxPublishPeriod { get; set; }
}