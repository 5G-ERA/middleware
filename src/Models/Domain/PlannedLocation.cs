using Middleware.Models.Enums;

namespace Middleware.Models.Domain;

public record PlannedLocation
{
    public Guid Id { get; init; }
    public string Name { get; init; } = default!;
    public LocationType Type { get; init; }
    public string? NetworkSliceName { get; }
    public bool HasSlicesEnabled { get; init; }

    public PlannedLocation(Guid id, string name, LocationType type)
    {
        Id = id;
        Name = name;
        Type = type;
    }

    public PlannedLocation(Guid id, string name, LocationType type, string networkSliceName)
    {
        Id = id;
        Name = name;
        Type = type;
        NetworkSliceName = networkSliceName;
        HasSlicesEnabled = true;
    }
}