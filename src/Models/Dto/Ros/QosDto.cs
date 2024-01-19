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
        var dto = this;
        return new()
        {
            Deadline = dto.Deadline,
            Depth = dto.Depth,
            Durability = dto.Durability,
            History = dto.History,
            Lease = dto.Lease,
            Lifespan = dto.Lifespan,
            Liveliness = dto.Liveliness,
            Preset = dto.Preset,
            Reliability = dto.Reliability
        };
    }
}