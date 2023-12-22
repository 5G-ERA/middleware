using Middleware.Models.Domain.Contracts;
using Middleware.Models.Domain.ValueObjects;
using Middleware.Models.Enums;

namespace Middleware.Models.Domain;

public record PlannedLocation
{
    public Guid Id { get; init; }
    public string Name { get; init; } = default!;
    public LocationType Type { get; init; }
    public string? NetworkSliceName { get; }
    public bool HasSlicesEnabled { get; init; }
    public HardwareSpec HardwareSpec { get; init; }
    public IHardwareRequirementClaim? HardwareClaim { get; init; }

    public PlannedLocation(Guid id, string name, LocationType type, HardwareSpec hardwareSpec,
        IHardwareRequirementClaim? hardwareClaim)
    {
        Id = id;
        Name = name;
        Type = type;
        HardwareClaim = hardwareClaim;
        HardwareSpec = hardwareSpec;
    }

    public PlannedLocation(Guid id, string name, LocationType type, string networkSliceName,
        HardwareSpec hardwareSpec, IHardwareRequirementClaim? hardwareClaim)
    {
        Id = id;
        Name = name;
        Type = type;
        NetworkSliceName = networkSliceName;
        HardwareClaim = hardwareClaim;
        HardwareSpec = hardwareSpec;
        HasSlicesEnabled = true;
    }
}