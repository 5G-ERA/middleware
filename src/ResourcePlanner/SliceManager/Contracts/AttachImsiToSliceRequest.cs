namespace Middleware.ResourcePlanner.SliceManager;

public record AttachImsiToSliceRequest
{
    public string Imsi { get; init; }
    public string SliceId { get; init; }
    public int UsedDataRateUl { get; init; }
    public int UsedDataRateDl { get; init; }
}