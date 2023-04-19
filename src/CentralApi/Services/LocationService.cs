using System.Collections.Immutable;
using FluentValidation;
using Middleware.CentralApi.Domain;
using Middleware.DataAccess.Repositories.Abstract;
using Middleware.Models.Domain;
using OneOf;
using OneOf.Types;
using Middleware.Models.Enums;

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
        var (queryResultCloud, cloud) = await _cloudRepository.CheckIfNameExists(location.Name);
        var (queryResultEdge, edge) = await _edgeRepository.CheckIfNameExists(location.Name);

        if ((queryResultEdge == false || edge is null) && (queryResultCloud == false || cloud is null))
        {
            return await RegisterNewLocation(location);
        }

        // make it online & return info about location based on matched edge
        if (cloud is null)
        {
            edge!.IsOnline = true;
            var result = new Location()
            {
                Id = edge.Id,
                Name = edge.Name,
                Organization = edge.Organization,
                Address = edge.EdgeIp,
                Type = edge.Type
            };
            await _edgeRepository.AddAsync(edge);
            return result;
        }
        else  // make it online & return info about location based on matched cloud
        {
            cloud.IsOnline = true;
            var result = new Location()
            {
                Id = cloud.Id,
                Name = cloud.Name,
                Organization = cloud.Organization,
                Address = cloud.CloudIp,
                Type = cloud.Type
            };
            await _cloudRepository.AddAsync(cloud);
            return result;
        }

    }

    private async Task<Location> RegisterNewLocation(Location location)
    {
        var id = Guid.NewGuid();
        if (location.Type == LocationType.Cloud)
        {
            var cloud = new CloudModel()
            {
                Id = id,
                Name = location.Name,
                Organization = location.Organization,
                LastUpdatedTime = DateTimeOffset.Now.Date
            };
            await _cloudRepository.AddAsync(cloud);
        }
        else if (location.Type == LocationType.Edge)
        {
            var cloud = new EdgeModel()
            {
                Id = id,
                Name = location.Name,
                Organization = location.Organization,
                LastUpdatedTime = DateTimeOffset.Now.Date
            };
            await _edgeRepository.AddAsync(cloud);
        }

        var newLoc = new Location()
        {
            Id = id,
            Name = location.Name,
            Organization = location.Organization,
            Type = location.Type
        };
        return newLoc;
    }

    public async Task<OneOf<ImmutableList<Location>, NotFound>> GetAvailableLocations(string organization)
    {
        // get all online edges and clouds
        var locations = new List<Location>();

        // edges where organization = organization
        var edges = await _edgeRepository.GetEdgesByOrganizationAsync(organization);

        var locationsEdges = edges.Select(x => new Location()
        {
            Id = x.Id,
            Type = LocationType.Edge,
            Name = x.Name,
            Address = x.EdgeIp,
            Organization = x.Organization
        });

        // clouds where organization = organization
        var clouds = await _cloudRepository.GetCloudsByOrganizationAsync(organization);

        var locationsClouds = clouds.Select(x => new Location()
        {
            Id = x.Id,
            Type = LocationType.Cloud,
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

        return locations.ToImmutableList();
    }
}