using Middleware.Models.Enums;

namespace Middleware.Models.Domain;

public record PlannedLocation
{
    public string Name { get; init; } = default!;
    public LocationType Type { get; init; }
    public string? NetworkSliceName { get; }
    public bool HasSlicesEnabled { get; init; }

    public PlannedLocation(string name, LocationType type)
    {
        Name = name;
        Type = type;
    }

    public PlannedLocation(string name, LocationType type, string networkSliceName)
    {
        Name = name;
        Type = type;
        NetworkSliceName = networkSliceName;
        HasSlicesEnabled = true;
    }
}