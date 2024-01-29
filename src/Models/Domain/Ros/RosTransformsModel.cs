using System.Text.Json.Serialization;
using Middleware.Models.Dto.Ros;

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

    public RosTransforms ToDto()
    {
        var domain = this;
        return new()
        {
            SourceFrame = domain.SourceFrame,
            AngularThres = domain.AngularThres,
            TargetFrame = domain.TargetFrame,
            TransThres = domain.TransThres,
            MaxPublishPeriod = domain.MaxPublishPeriod
        };
    }
}