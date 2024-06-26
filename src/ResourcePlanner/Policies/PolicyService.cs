﻿using Middleware.Models.Domain;
using Middleware.Models.Enums;
using Middleware.RedisInterface.Contracts.Mappings;
using Middleware.RedisInterface.Sdk;
using Middleware.ResourcePlanner.Policies.LocationSelection;

namespace Middleware.ResourcePlanner.Policies;

internal class PolicyService : IPolicyService
{
    private readonly ILogger<PolicyService> _logger;
    private readonly IPolicyBuilder _policyBuilder;
    private readonly IRedisInterfaceClient _redisInterfaceClient;

    public PolicyService(IPolicyBuilder policyBuilder, IRedisInterfaceClient redisInterfaceClient,
        ILogger<PolicyService> logger)
    {
        _policyBuilder = policyBuilder;
        _redisInterfaceClient = redisInterfaceClient;
        _logger = logger;
    }

    /// <inheritdoc />
    public async Task<PlannedLocation> GetLocationAsync(IReadOnlyList<InstanceModel> members)
    {
        var allAppliedPolicies = new HashSet<string>();
        var resultLocations = new HashSet<PlannedLocation>();
        var resourcePolicy = await GetResourceLocationPolicy();
        // BB: should not happen but who knows...
        if (resourcePolicy is null)
        {
            _logger.LogCritical("System policy not found {0}", nameof(ResourceBasedLocation));
            return null;
        }

        foreach (var member in members)
        {
            if (resourcePolicy.IsActive)
                member.AppliedPolicies.Add(PolicyBuilder.ResourcePolicyName);

            if (member.AppliedPolicies.Any() == false)
            {
                var policy = _policyBuilder.GetDefaultLocationPolicy();
                var location = await policy.GetLocationAsync(member);
                resultLocations.Add(location);
                continue;
            }

            var localLocations = new HashSet<Tuple<Priority, PlannedLocation>>();
            foreach (var policyName in member.AppliedPolicies)
            {
                allAppliedPolicies.Add(policyName);
                var policy = await _policyBuilder.CreateLocationPolicy(policyName);
                if (policy is null)
                    continue;

                var location = await policy.GetLocationAsync(member);
                if (location is not null)
                    localLocations.Add(new(policy.Priority, location));
            }

            // always 1 location will match, if more, negotiate the best location for instance
            var desiredLocation = localLocations.Count == 1
                ? localLocations.First().Item2
                : await NegotiateLocation(localLocations, member.AppliedPolicies);

            resultLocations.Add(desiredLocation);
        }

        if (resultLocations.Count == 1)
            return resultLocations.First();

        // negotiate action-level location
        var locationsTmp = resultLocations.Select(x => new Tuple<Priority, PlannedLocation>(Priority.None, x))
            .ToHashSet();
        var negotiatedLocation = await NegotiateLocation(locationsTmp, allAppliedPolicies.ToList());
        return negotiatedLocation;
    }

    private async Task<PolicyModel> GetResourceLocationPolicy()
    {
        var response = await _redisInterfaceClient.GetPolicyByNameAsync("ResourceBasedLocation");

        return response.ToPolicy();
    }

    /// <summary>
    ///     Checks what locations from the specified set, meet the most of the applied policies
    /// </summary>
    /// <param name="locations"></param>
    /// <param name="policyNames"></param>
    /// <returns></returns>
    private async Task<PlannedLocation> NegotiateLocation(IReadOnlySet<Tuple<Priority, PlannedLocation>> locations,
        IReadOnlyList<string> policyNames)
    {
        PlannedLocation retVal = null;
        var locationPolicyHierarchy = new List<Tuple<int, Priority, PlannedLocation>>();
        foreach (var (priority, location) in locations.OrderByDescending(x => (int)x.Item1))
        {
            var meetsAll = true;
            var cnt = 0;
            foreach (var policy in policyNames)
            {
                var policyImpl = await _policyBuilder.CreateLocationPolicy(policy);

                if (policyImpl.FoundMatchingLocation == false) continue;

                var meets = await policyImpl.IsLocationSatisfiedByPolicy(location);
                if (meets)
                    cnt++;
                meetsAll &= meets;
            }

            locationPolicyHierarchy.Add(new(cnt, priority, location));

            if (!meetsAll)
                continue;

            retVal = location;
            break;
        }

        return retVal ?? await GetDesiredLocationFromHierarchy(locationPolicyHierarchy);
    }

    /// <summary>
    ///     Resolves the hierarchy of the locations to select one that matches the most of the policyNames applied
    /// </summary>
    /// <param name="hierarchy"></param>
    /// <returns>The most applicable location in accordance to the policyNames used</returns>
    private async Task<PlannedLocation> GetDesiredLocationFromHierarchy(
        IEnumerable<Tuple<int, Priority, PlannedLocation>> hierarchy)
    {
        var ordered = hierarchy.OrderByDescending(x => x.Item1)
            .ThenByDescending(x => (int)x.Item2)
            .ToList();
        var max = ordered.FirstOrDefault()?.Item1;

        // less or equal to one because a location selected by policy will always match itself
        if (max <= 1) return ordered.First().Item3;

        // when no location can be selected, we return the default location
        var defaultLocation = _policyBuilder.GetDefaultLocationPolicy();
        return await defaultLocation.GetLocationAsync(null!);
    }
}