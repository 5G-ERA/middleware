using Middleware.Models.Domain.Ros;

namespace Middleware.Models.Dto.Ros;

public class QosDto
{
    public string? Preset { get; set; } = default!;
    public string? History { get; set; } = default!;
    public int? Depth { get; set; }
    public string? Reliability { get; set; } = default!;
    public string? Durability { get; set; } = default!;
    public string? Deadline { get; set; } = default!;
    public string? Lifespan { get; set; } = default!;
    public string? Liveliness { get; set; } = default!;
    public string? Lease { get; set; } = default!;

    public Qos ToModel()
    {
        return new()
        {
            Deadline = Deadline,
            Depth = Depth,
            Durability = Durability,
            History = History,
            Lease = Lease,
            Lifespan = Lifespan,
            Liveliness = Liveliness,
            Preset = Preset,
            Reliability = Reliability
        };
    }
}