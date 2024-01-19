namespace Middleware.RedisInterface.Contracts.Responses;

public class QosResponse
{
    public string Preset { get; set; } = default!;
    public string History { get; set; } = default!;
    public int? Depth { get; set; } = default!;
    public string Reliability { get; set; } = default!;
    public string Durability { get; set; } = default!;
    public string Deadline { get; set; } = default!;
    public string Lifespan { get; set; } = default!;
    public string Liveliness { get; set; } = default!;
    public string Lease { get; set; } = default!;
}