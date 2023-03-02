using Middleware.Common.Enums;

namespace Middleware.CentralApi.Domain;

public class Location
{
    public Guid Id { get; init; }
    public LocationType Type { get; init; }
    public string Name { get; init; }
    public Uri Address { get; init; }
}