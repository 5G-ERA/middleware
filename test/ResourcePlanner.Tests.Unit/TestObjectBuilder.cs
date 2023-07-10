using Middleware.CentralApi.Contracts.Responses;
using Middleware.Models.Domain;
using Middleware.Models.ExtensionMethods;

namespace ResourcePlanner.Tests.Unit;

public static class TestObjectBuilder
{
    public static LocationsResponse ExampleLocationsResponse(params BaseModel[] locations)
    {
        return new()
        {
            Locations = locations.Select(l => new LocationResponse
            {
                Name = l.Name,
                Id = l.Id,
                Type = l.GetType().GetModelName()
            })
        };
    }
}