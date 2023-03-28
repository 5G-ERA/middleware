namespace Middleware.CentralApi.Contracts.Responses;

public class LocationsResponse
{
    public IEnumerable<LocationResponse> Locations { get; init; } = Enumerable.Empty<LocationResponse>();
}