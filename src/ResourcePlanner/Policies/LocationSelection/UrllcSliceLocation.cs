﻿using System.Collections.Immutable;
using Middleware.CentralApi.Sdk;
using Middleware.Common.Enums;
using Middleware.Models.Domain;
using Middleware.Models.Domain.Contracts;
using Middleware.Models.Domain.ValueObjects;
using Middleware.Models.Enums;
using Middleware.RedisInterface.Contracts.Mappings;
using Middleware.RedisInterface.Sdk;

namespace Middleware.ResourcePlanner.Policies.LocationSelection;

internal class UrllcSliceLocation : ILocationSelectionPolicy
{
    private readonly ICentralApiClient _centralApi;
    private readonly ILogger _logger;
    private readonly IRedisInterfaceClient _redisInterfaceClient;

    public UrllcSliceLocation(Priority priority, IRedisInterfaceClient redisInterfaceClient,
        ICentralApiClient centralApi, ILogger logger)
    {
        _redisInterfaceClient = redisInterfaceClient;
        _centralApi = centralApi;
        _logger = logger;
        Priority = priority;
    }

    /// <inheritdoc />
    public Priority Priority { get; }

    /// <inheritdoc />
    public bool FoundMatchingLocation { get; private set; }

    /// <inheritdoc />
    public async Task<bool> IsLocationSatisfiedByPolicy(PlannedLocation plannedLocation)
    {
        var location = new Location
        {
            Id = plannedLocation.Id,
            Name = plannedLocation.Name,
            Type = plannedLocation.Type
        };

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

    /// <inheritdoc />
    public async Task<PlannedLocation> GetLocationAsync(IHardwareRequirementClaim hwClaim = null)
    {
        var slices = await _redisInterfaceClient.SliceGetAllAsync();
        if (slices is null)
            return null;

        var urllcSlices = slices.ToSliceList().Where(t => t.SliceType == SliceType.Urllc).ToImmutableList();
        if (urllcSlices.Count == 0)
            return null;

        var availableLocationsResponse = await _centralApi.GetAvailableLocations();
        if (availableLocationsResponse is null)
            return null;

        var availableLocationIds = availableLocationsResponse.Locations.Select(l => l.Id).ToList();

        //TODO: include the lowest latency to the location available to the robot
        var bestSlice = urllcSlices.MinBy(s => s.Latency);

        var relations =
            await _redisInterfaceClient.GetRelationAsync(bestSlice, "OFFERS", RelationDirection.Incoming.ToString());


        var location = relations?
            .Where(l => availableLocationIds.Contains(l.InitiatesFrom.Id))
            .FirstOrDefault();

        if (location is null)
            return null;

        FoundMatchingLocation = true;
        var locDetails = availableLocationsResponse.Locations.FirstOrDefault(l => l.Id == location.InitiatesFrom.Id);
        var loc = await _redisInterfaceClient.GetLocationByIdAsync(locDetails!.Id);
        if (loc is null) return null;

        var hwSpec = new HardwareSpec
        {
            Ram = loc.Ram,
            Cpu = loc.Cpu,
            NumberCores = loc.NumberOfCores,
            StorageDisk = loc.DiskStorage,
            VirtualRam = loc.VirtualRam,
            Throughput = loc.Throughput,
            Latency = loc.Latency
        };
        var type = Enum.Parse<LocationType>(locDetails.Type, true);
        return new(location.InitiatesFrom.Id, location.InitiatesFrom.Name, type, bestSlice.Name, hwSpec, null);
    }
}