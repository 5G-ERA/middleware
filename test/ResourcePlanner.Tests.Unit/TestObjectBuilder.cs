using Middleware.CentralApi.Contracts.Responses;
using Middleware.Models.Domain.Contracts;

namespace ResourcePlanner.Tests.Unit;

public static class TestObjectBuilder
{
    public static LocationsResponse ExampleLocationsResponse(params ILocation[] locations)
    {
        return new()
        {
            Locations = locations.Select(l => new LocationResponse
            {
                Name = l.Name,
                Id = l.Id,
                Type = l.Type.ToString()
            })
        };
    }
}