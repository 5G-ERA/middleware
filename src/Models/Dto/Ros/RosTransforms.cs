using Microsoft.VisualBasic.CompilerServices;
using Middleware.Models.Domain.Ros;

namespace Middleware.Models.Dto.Ros;

public class RosTransforms
{
    public string SourceFrame { get; set; } = default!;
    public string TargetFrame { get; set; } = default!;
    public double AngularThres { get; set; }
    public double TransThres { get; set; }
    public double MaxPublishPeriod { get; set; }

    public RosTransformsModel ToModel()
    {
        var dto = this;
        return new()
        {
            AngularThres = dto.AngularThres,
            SourceFrame = dto.SourceFrame,
            TargetFrame = dto.TargetFrame,
            TransThres = dto.TransThres,
            MaxPublishPeriod = dto.MaxPublishPeriod
        };
    }
}