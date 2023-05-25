using Middleware.CentralApi.Contracts.Responses;
using Middleware.Models.Domain;

namespace Middleware.CentralApi.Mappings;

public static class DomainToApiContractMapper
{
    public static LocationResponse ToLocationResponse(this Location x)
    {
        return new()
        {
            Id = x.Id!.Value,
            Name = x.Name,
            Organization = x.Organization,
            Type = x.Type.ToString(),
            IsOnline = true,
            Address = x.Address
        };
    }

    public static LocationsResponse ToLocationsResponse(this IEnumerable<Location> locations)
    {
        return new()
        {
            Locations = locations.Select(x => x.ToLocationResponse())
        };
    }
}