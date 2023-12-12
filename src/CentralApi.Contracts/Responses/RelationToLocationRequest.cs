namespace Middleware.CentralApi.Contracts.Responses;

public class RelationToLocationRequest
{
    public Guid robotId { get; set; }

    public List<LocationNames> Locations { get; set; }
}
public class LocationNames
{
    public string? LocationName { get; set; }

    public string? OrganisationName { get; set; }
}