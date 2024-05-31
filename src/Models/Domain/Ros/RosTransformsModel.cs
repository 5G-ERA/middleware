using System.Text.Json.Serialization;
using Middleware.Models.Dto.Ros;

namespace Middleware.Models.Domain.Ros;

public class RosTransformsModel
{
    public string SourceFrame { get; set; } = default!;

    public string TargetFrame { get; set; } = default!;

    public double AngularThres { get; set; }

    public double TransThres { get; set; }

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