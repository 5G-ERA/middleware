using System.Collections.Immutable;
using Middleware.Common.Enums;
using Middleware.DataAccess.Repositories.Abstract;
using Middleware.Models.Domain;
using Middleware.Models.Dto;
using Redis.OM.Contracts;
using RedisGraphDotNet.Client;
using Serilog;

namespace Middleware.DataAccess.Repositories;

public class RedisLocationRepository : RedisRepository<Location, LocationDto>, ILocationRepository
{
    /// <inheritdoc />
    public RedisLocationRepository(IRedisConnectionProvider provider, IRedisGraphClient redisGraph, ILogger logger) :
        base(provider, redisGraph, true, logger)
    {
    }

    /// <inheritdoc />
    public async Task<Location?> PatchAsync(Guid id, Location patch)
    {
        var currentModel = await GetByIdAsync(id);
        if (currentModel == null) return null;
        if (!string.IsNullOrEmpty(patch.Name)) currentModel.Name = patch.Name;
        if (!string.IsNullOrEmpty(patch.Status)) currentModel.Status = patch.Status;
        if (Uri.IsWellFormedUriString(patch.Address.ToString(), UriKind.RelativeOrAbsolute))
            currentModel.Address = patch.Address;
        if (!string.IsNullOrEmpty(patch.MacAddress)) currentModel.MacAddress = patch.MacAddress;
        if (patch.Cpu is not null && patch.Cpu > 0) currentModel.Cpu = patch.Cpu;
        if (patch.NumberOfCores is not null && patch.NumberOfCores >= 0)
            currentModel.NumberOfCores = patch.NumberOfCores;
        if (patch.Ram is not null && patch.Ram > 0) currentModel.Ram = patch.Ram;
        if (patch.VirtualRam is not null && patch.VirtualRam > 0) currentModel.VirtualRam = patch.VirtualRam;
        if (patch.DiskStorage is not null && patch.DiskStorage > 0) currentModel.DiskStorage = patch.DiskStorage;

        currentModel.LastUpdatedTime = DateTimeOffset.UtcNow.DateTime;

        await UpdateAsync(currentModel);
        return currentModel;
    }

    /// <inheritdoc />
    public async Task<Location?> GetByNameAsync(string name)
    {
        var loc = await FindSingleAsync(dto => dto.Name == name);
        return loc;
    }

    /// <inheritdoc />
    public async Task<List<Location>> FilterFreeLocationsAsync(IReadOnlyList<Location> locationsToCheck)
    {
        var freeLocations = new List<Location>();

        foreach (var loc in locationsToCheck)
        {
            var relations = await GetRelation(
                loc.Id,
                "LOCATED_AT",
                RelationDirection.Incoming);

            if (relations.Count == 0) freeLocations.Add(loc);
        }

        return freeLocations;
    }

    /// <inheritdoc />
    public async Task<int> GetDeployedInstancesCountAsync(string name)
    {
        var edge = await FindSingleAsync(dto => dto.Name == name);
        if (edge is null)
            throw new ArgumentException("Location does not exist", nameof(name));

        var robotRelations = await GetRelation(edge.Id, "LOCATED_AT", RelationDirection.Incoming);
        return robotRelations.Count;
    }

    /// <inheritdoc />
    public async Task<int> GetDeployedInstancesCountAsync(Guid id)
    {
        var loc = await GetByIdAsync(id);
        if (loc is null)
            throw new ArgumentException("Location does not exist", nameof(id));
        var relations = await GetRelation(id, "LOCATED_AT", RelationDirection.Incoming);
        return relations.Count;
    }

    /// <inheritdoc />
    public async Task<bool> IsBusyAsync(string name)
    {
        var loc = await FindSingleAsync(dto => dto.Name == name);
        if (loc is null)
            throw new ArgumentException("Location does not exist", nameof(name));

        var relations = await GetRelation(loc.Id, "LOCATED_AT", RelationDirection.Incoming);
        return relations.Count > 0;
    }

    /// <inheritdoc />
    public async Task<bool> IsBusyAsync(Guid id)
    {
        var loc = await GetByIdAsync(id);
        if (loc is null)
            throw new ArgumentException("Location does not exist", nameof(id));

        var relations = await GetRelation(id, "LOCATED_AT", RelationDirection.Incoming);
        return relations.Count > 0;
    }

    /// <inheritdoc />
    public async Task<ImmutableList<Location>> GetLocationsByOrganizationAsync(string organization)
    {
        var locations = await FindAsync(dto => dto.Organization == organization);
        return locations.ToImmutableList();
    }

    /// <inheritdoc />
    public async Task<Location?> GetSingleLocationByOrganizationAndNameAsync(string organization, string name)
    {
        var location = await FindSingleAsync(dto => dto.Organization == organization && dto.Name == name);
        return location;
    }

    /// <inheritdoc />
    public async Task<(bool, Location?)> ExistsAsync(string name)
    {
        var loc = await FindSingleAsync(dto => dto.Name == name);

        return (loc is not null, loc);
    }

    /// <inheritdoc />
    public async Task<(bool, Location?)> ExistsAsync(Guid id)
    {
        var loc = await GetByIdAsync(id);

        return (loc is not null, loc);
    }

    /// <inheritdoc />
    public async Task<List<Location>> OrderLocationsByUtilizationAsync(List<Location> locationsToCheck,
        bool descending = false)
    {
        var counter = new Dictionary<Location, int>();

        foreach (var busyEdge in locationsToCheck)
        {
            var robotRelations =
                await GetRelation(busyEdge.Id, "LOCATED_AT", RelationDirection.Incoming);

            counter.Add(busyEdge, robotRelations.Count);
        }

        var ordered = descending
            ? locationsToCheck.OrderByDescending(l => counter[l])
            : locationsToCheck.OrderBy(l => counter[l]);

        return ordered.ToList();
    }
}