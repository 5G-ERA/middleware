namespace Middleware.RedisInterface.Contracts.Responses;

public class RosTransformsResponse
{
    public string SourceFrame { get; set; } = default!;
    public string TargetFrame { get; set; } = default!;
    public double AngularThres { get; set; }
    public double TransThres { get; set; }
    public double MaxPublishPeriod { get; set; }
}