using Middleware.Models.Enums;

namespace Middleware.Models.Domain.ValueObjects;

public class NetAppRequirement
{
    /// <summary>
    ///     Value for the minimal requirement for the resource from the NetApp
    /// </summary>
    public long Minimal { get; set; }

    /// <summary>
    ///     Value for the optimal work of the NetApp
    /// </summary>
    public long Optimal { get; set; }

    /// <summary>
    ///     Priority describing how important
    /// </summary>
    public ResourcePriority Priority { get; set; }

    /// <summary>
    ///     Are values ordered in Ascending order
    /// </summary>
    public bool Ascending { get; set; }

    public NetAppRequirement(long minimal, long optimal, ResourcePriority priority, bool ascending = true)
    {
        Minimal = minimal;
        Optimal = optimal;
        Priority = priority;
        Ascending = ascending;
    }

    public NetAppRequirement(long minimal, long optimal, string priority, bool ascending = true)
    {
        Minimal = minimal;
        Optimal = optimal;
        Priority = Enum.Parse<ResourcePriority>(priority);
        Ascending = ascending;
    }

    public NetAppRequirement(long? minimal, long? optimal, string? priority, bool ascending = true)
    {
        if (minimal != null) Minimal = minimal.Value;
        if (optimal != null) Optimal = optimal.Value;
        if (priority != null) Priority = Enum.Parse<ResourcePriority>(priority);
        Ascending = ascending;
    }
}