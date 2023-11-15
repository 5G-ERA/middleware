namespace Middleware.RedisInterface.Contracts.Responses;

public class OrgStructureResponse
{
    public IEnumerable<LocationResponse> Locations { get; init; } = Enumerable.Empty<LocationResponse>();
}