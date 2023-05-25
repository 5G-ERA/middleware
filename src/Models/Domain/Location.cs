using Middleware.Models.Enums;

namespace Middleware.Models.Domain;

public class Location
{
    public Guid? Id { get; init; }
    public LocationType Type { get; init; }
    public string Name { get; init; } = default!;
    public Uri? Address { get; init; } = default!;
    public string Organization { get; init; } = default!;
}