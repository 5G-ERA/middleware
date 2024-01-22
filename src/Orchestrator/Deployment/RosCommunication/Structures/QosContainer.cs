using Middleware.Models.Domain.Ros;

namespace Middleware.Orchestrator.Deployment.RosCommunication.Structures;

internal class QosContainer
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

    public static QosContainer FromQosModel(Qos x)
    {
        return new()
        {
            Deadline = x.Deadline,
            Depth = x.Depth,
            Durability = x.Durability,
            History = x.History,
            Lease = x.Lease,
            Lifespan = x.Lifespan,
            Liveliness = x.Liveliness,
            Reliability = x.Reliability,
            Preset = x.Preset
        };
    }
}