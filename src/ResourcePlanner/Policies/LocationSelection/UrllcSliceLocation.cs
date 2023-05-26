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

        var urllcSlices = slices.ToSliceList().Where(t => t.SliceType == SliceType.Urllc).ToImmutableList();

        //TODO: include the lowest latency to the location available to the robot
        var bestSlice = urllcSlices.MinBy(s => s.Latency);

        var relations =
            await _redisInterfaceClient.GetRelationAsync(bestSlice, "OFFERS", RelationDirection.Incoming.ToString());

        var location = relations?.First();

        if (location is null)
            return null;

        FoundMatchingLocation = true;
        var type = Enum.Parse<LocationType>(location.InitiatesFrom.Type);
        return new(location.InitiatesFrom.Name, type, bestSlice.Name);
    }

    /// <inheritdoc />
    public Task<bool> IsLocationSatisfiedByPolicy(PlannedLocation plannedLocation)
    {
        throw new NotImplementedException();
    }
}