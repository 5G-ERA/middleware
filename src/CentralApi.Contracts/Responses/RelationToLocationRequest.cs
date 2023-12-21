namespace Middleware.CentralApi.Contracts.Responses;

public class RelationToLocationRequest
{
    public Guid RobotId { get; set; }

    public List<LocationNames> Locations { get; set; } = new List<LocationNames>();
}
public class LocationNames
{
    public string? LocationName { get; set; }

    public string? OrganisationName { get; set; }
}