using FluentValidation;
using Middleware.CentralApi.Domain;
using Middleware.DataAccess.Repositories.Abstract;
using Middleware.Models.Domain;
using OneOf;
using OneOf.Types;
using Middleware.CentralApi.Mappings;
using Middleware.Common.Enums;

namespace Middleware.CentralApi.Services;

public class LocationService : ILocationService
{
    private readonly ICloudRepository _cloudRepository;
    private readonly IEdgeRepository _edgeRepository;


    public LocationService(ICloudRepository cloudRepository, IEdgeRepository edgeRepository)
    {
        _cloudRepository = cloudRepository;
        _edgeRepository = edgeRepository;
    }

    public async Task<OneOf<Location, ValidationException, NotFound>> RegisterLocation(Location location)
    {
        // when location not found in db
        
        if (false)
        {
            return new NotFound();    
        }

        BaseModel locationData = null;
        // when location is not valid eg. different type than in the system etc
        if (false)
        {
            return new ValidationException("The specified location is not valid");    
        }
        
        // make it online
        var result = new Location()
        {
            Id = locationData.Id,
            Name = location.Name,
            Organization = location.Organization,
            Type = location.Type
        };
        // when ok
        return result;

    }

    public async Task<OneOf<List<Location>, NotFound>> GetAvailableLocations(string organization)
    {
        // get all online edges and clouds
        var locations = new List<Location>();

        // edges where organization = organization
        List<EdgeModel> edges = await _edgeRepository.GetEdgesByOrganizationAsync(organization);

        var locationsEdges = edges.Select(x => new Location()
        {
            Id = x.Id,
            Type = Enum.Parse<LocationType>(x.Type),
            Name = x.Name,
            Address = x.EdgeIp,
            Organization = x.Organization
        });

        // clouds where organization = organization
        List<CloudModel> clouds = await _cloudRepository.GetCloudsByOrganizationAsync(organization);

        var locationsClouds = clouds.Select(x => new Location()
        {
            Id = x.Id,
            Type = Enum.Parse<LocationType>(x.Type),
            Name = x.Name,
            Address = x.CloudIp,
            Organization = x.Organization
        });

        locations.AddRange(locationsEdges);
        locations.AddRange(locationsClouds);

        if (locations.Any() == false)
        {
            return new NotFound();
        }

        return locations;
    }
}