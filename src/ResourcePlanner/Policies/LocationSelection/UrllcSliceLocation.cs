using System.Collections.Immutable;
using Middleware.Common.Enums;
using Middleware.Models.Domain;
using Middleware.Models.Enums;
using Middleware.RedisInterface.Contracts.Mappings;
using Middleware.RedisInterface.Sdk;

namespace Middleware.ResourcePlanner.Policies.LocationSelection;

internal class UrllcSliceLocation : ILocationSelectionPolicy
{
    private readonly IRedisInterfaceClient _redisInterfaceClient;

    public UrllcSliceLocation(Priority priority, IRedisInterfaceClient redisInterfaceClient)
    {
        _redisInterfaceClient = redisInterfaceClient;
        Priority = priority;
    }

    /// <inheritdoc />
    public Priority Priority { get; }

    /// <inheritdoc />
    public bool FoundMatchingLocation { get; private set; }

    /// <inheritdoc />
    public async Task<PlannedLocation> GetLocationAsync()
    {
        var slices = await _redisInterfaceClient.SliceGetAllAsync();
        if (slices is null)
            return null;

        var urllcSlices = slices.ToSliceList().Where(t => t.SliceType == SliceType.Urllc).ToImmutableList();
        if (urllcSlices.Count == 0)
            return null;

        //TODO: include the lowest latency to the location available to the robot
        var bestSlice = urllcSlices.MinBy(s => s.Latency);

        var relations =
            await _redisInterfaceClient.GetRelationAsync(bestSlice, "OFFERS", RelationDirection.Incoming.ToString());

        var location = relations?.FirstOrDefault();

        if (location is null)
            return null;

        FoundMatchingLocation = true;
        var type = Enum.Parse<LocationType>(location.InitiatesFrom.Type, true);
        return new(location.InitiatesFrom.Id, location.InitiatesFrom.Name, type, bestSlice.Name);
    }

    /// <inheritdoc />
    public async Task<bool> IsLocationSatisfiedByPolicy(PlannedLocation plannedLocation)
    {
        BaseModel location;
        if (plannedLocation.Type == LocationType.Cloud)
        {
            location = new CloudModel
            {
                Id = plannedLocation.Id,
                Name = plannedLocation.Name
            };
        }
        else
        {
            location = new EdgeModel
            {
                Id = plannedLocation.Id,
                Name = plannedLocation.Name
            };
        }

        var relations = await _redisInterfaceClient.GetRelationAsync(location, "OFFERS");

        if (relations is null) return false;

        foreach (var relation in relations)
        {
            var sliceResponse = await _redisInterfaceClient.SliceGetByIdAsync(relation.PointsTo.Id);
            if (sliceResponse is null)
                continue;

            var slice = sliceResponse.ToSlice();
            if (slice.SliceType == SliceType.Urllc)
                return true;
        }

        return false;
    }
}