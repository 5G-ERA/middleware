using System.Collections.Immutable;
using FluentValidation;
using Middleware.DataAccess.Repositories.Abstract;
using Middleware.Models.Domain;
using OneOf;
using OneOf.Types;

namespace Middleware.CentralApi.Services;

public class LocationService : ILocationService
{
    private readonly ILocationRepository _locationRepository;


    public LocationService(ILocationRepository locationRepository)
    {
        _locationRepository = locationRepository;
    }

    public async Task<OneOf<Location, ValidationException, NotFound>> RegisterLocation(Location location)
    {
        // when location not found in db
        var (found, loc) = await _locationRepository.ExistsAsync(location.Name);


        if (found == false)
            return await RegisterNewLocation(location);

        // make it online & return info about location based on matched edge
        loc!.IsOnline = true;
        loc!.Address = location.Address;
        await _locationRepository.UpdateAsync(loc);
        return loc;
    }

    public async Task<OneOf<ImmutableList<Location>, NotFound>> GetAvailableLocations(string organization)
    {
        // get all online edges and clouds where organization = organization
        var edges = await _locationRepository.GetLocationsByOrganizationAsync(organization);

        if (edges.Any() == false) return new NotFound();

        return edges;
    }

    private async Task<Location> RegisterNewLocation(Location location)
    {
        location.LastUpdatedTime = DateTimeOffset.Now.DateTime;
        await _locationRepository.AddAsync(location);
        return location;
    }
}