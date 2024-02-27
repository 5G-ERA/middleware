using Middleware.CentralApi.Contracts;
using Middleware.CentralApi.Sdk;
using Middleware.Models.Domain;
using Middleware.Models.Domain.Contracts;
using Middleware.Models.Domain.ValueObjects;
using Middleware.Models.Enums;
using Middleware.RedisInterface.Contracts.Mappings;
using Middleware.RedisInterface.Sdk;

namespace Middleware.ResourcePlanner.Policies.LocationSelection;

internal class ResourceBasedLocation : ILocationSelectionPolicy
{
    private readonly ICentralApiClient _centralApiClient;
    private readonly ILogger _logger;
    private readonly IRedisInterfaceClient _redisInterfaceClient;

    public ResourceBasedLocation(Priority priority, IRedisInterfaceClient redisInterfaceClient,
        ICentralApiClient centralApiClient, ILogger logger)
    {
        _redisInterfaceClient = redisInterfaceClient;
        _centralApiClient = centralApiClient;
        _logger = logger;
        Priority = priority;
    }

    /// <inheritdoc />
    public Priority Priority { get; }

    /// <inheritdoc />
    public bool FoundMatchingLocation { get; set; }

    /// <inheritdoc />
    public async Task<PlannedLocation> GetLocationAsync(IHardwareRequirementClaim hwClaim = null)
    {
        var availableLocations = await GetAvailableLocations();
        var allLocations = await GetAllLocations();
        var availableIds = availableLocations.Select(l => l.Id).ToList();

        var location = allLocations.Where(loc => availableIds.Contains(loc.Id)).ToList();

        //pass the netapp requirements

        //calculate the colours and score for each location
        var scores = new Dictionary<Location, int>();
        foreach (var loc in location)
        {
            // what to do if the location does not have necessary data?
            // not include it
            var score = 0;
            if (hwClaim.Ram!.IsValid())
            {
                var ramScore = hwClaim.Ram.GetValueScore(loc.Ram!.Value);
                if (ramScore == ColourCode.Red)
                {
                    //score not calculated
                    continue;
                }

                score += (int)ramScore * (int)hwClaim.Ram.Priority;
            }

            if (hwClaim.NumberOfCores!.IsValid())
            {
                var coresScore = hwClaim.NumberOfCores.GetValueScore(loc.NumberOfCores!.Value);
                if (coresScore == ColourCode.Red)
                {
                    //score not calculated
                    continue;
                }

                score += (int)coresScore * (int)hwClaim.NumberOfCores.Priority;
            }

            if (hwClaim.DiskStorage!.IsValid())
            {
                var diskScore = hwClaim.DiskStorage.GetValueScore(loc.DiskStorage!.Value);
                if (diskScore == ColourCode.Red)
                {
                    //score not calculated
                    continue;
                }

                score += (int)diskScore * (int)hwClaim.DiskStorage.Priority;
            }

            if (hwClaim.Throughput!.IsValid())
            {
                var throughputScore = hwClaim.Throughput.GetValueScore(loc.Throughput!.Value);
                if (throughputScore == ColourCode.Red)
                {
                    //score not calculated
                    continue;
                }

                score += (int)throughputScore * (int)hwClaim.Throughput.Priority;
            }

            if (hwClaim.Latency!.IsValid())
            {
                var latScore = hwClaim.Latency.GetValueScore(loc.Latency!.Value);
                if (latScore == ColourCode.Red)
                {
                    //score not calculated
                    continue;
                }

                score += (int)latScore * (int)hwClaim.Latency.Priority;
            }
            
            if (score > 0)
            {
                scores.Add(loc, score);    
            }
        }

        KeyValuePair<Location, int>? maxScore = null;
        foreach (var score in scores)
        {
            maxScore ??= score;
            if (score.Value > maxScore.Value.Value) maxScore = score;
        }

        if (maxScore is null) return null;
        FoundMatchingLocation = true;
        var finalLoc = maxScore.Value.Key;
        var hwSpec = new HardwareSpec
        {
            Ram = finalLoc.Ram,
            Cpu = finalLoc.Cpu,
            NumberCores = finalLoc.NumberOfCores,
            StorageDisk = finalLoc.DiskStorage,
            VirtualRam = finalLoc.VirtualRam,
            Latency = finalLoc.Latency,
            Throughput = finalLoc.Throughput
        };
        return new(finalLoc.Id, finalLoc.Name, finalLoc.Type, hwSpec, hwClaim);
    }

    /// <inheritdoc />
    public Task<bool> IsLocationSatisfiedByPolicy(PlannedLocation plannedLocation)
    {
        if (plannedLocation.HardwareClaim is null) return Task.FromResult(true);

        var spec = plannedLocation.HardwareSpec;
        var claim = plannedLocation.HardwareClaim;

        if (claim.Ram!.IsValid())
        {
            var ramScore = claim.Ram.GetValueScore(spec.Ram!.Value);
            if (ramScore == ColourCode.Red) return Task.FromResult(false);
        }

        if (claim.NumberOfCores!.IsValid())
        {
            var coresScore = claim.NumberOfCores.GetValueScore(spec.NumberCores!.Value);
            if (coresScore == ColourCode.Red) return Task.FromResult(false);
        }

        if (claim.DiskStorage!.IsValid())
        {
            var diskScore = claim.DiskStorage.GetValueScore(spec.StorageDisk!.Value);
            if (diskScore == ColourCode.Red) return Task.FromResult(false);
        }

        if (claim.Throughput!.IsValid())
        {
            var throughputScore = claim.Throughput.GetValueScore(spec.Throughput!.Value);
            if (throughputScore == ColourCode.Red) return Task.FromResult(false);
        }

        if (claim.Latency!.IsValid())
        {
            var latScore = claim.Latency.GetValueScore(spec.Latency!.Value);
            if (latScore == ColourCode.Red) return Task.FromResult(false);
        }

        return Task.FromResult(true);
    }

    private async Task<List<Location>> GetAvailableLocations()
    {
        var availableLocationsResp = await _centralApiClient.GetAvailableLocations();
        if (availableLocationsResp is null) return null;
        return availableLocationsResp.Locations.Select(l => l.ToLocation()).ToList();
    }

    private async Task<List<Location>> GetAllLocations()
    {
        var availableLocationsResp = await _redisInterfaceClient.LocationGetAllAsync();
        if (availableLocationsResp is null) return null;
        return availableLocationsResp.ToLocations();
    }
}