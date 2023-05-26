using Microsoft.Extensions.Options;
using Middleware.Common.Config;
using Middleware.Models.Domain;
using Middleware.Models.Domain.Contracts;
using Middleware.Models.Enums;
using Middleware.ResourcePlanner.Policies.LocationSelection;

namespace Middleware.ResourcePlanner.Policies;

internal class PolicyService : IPolicyService
{
    private readonly IOptions<MiddlewareConfig> _middlewareConfig;
    private readonly IPolicyBuilder _policyBuilder;

    public PolicyService(IPolicyBuilder policyBuilder, IOptions<MiddlewareConfig> middlewareConfig)
    {
        _policyBuilder = policyBuilder;
        _middlewareConfig = middlewareConfig;
    }

    /// <inheritdoc />
    public async Task<PlannedLocation> GetLocationAsync(IReadOnlyList<IPolicyAssignable> members)
    {
        var allAppliedPolicies = new HashSet<string>();
        var resultLocations = new HashSet<PlannedLocation>();
        foreach (var member in members)
        {
            if (member.AppliedPolicies.Any() == false)
            {
                var policy = new DefaultLocation(_middlewareConfig);
                var location = await policy.GetLocationAsync();
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

                var location = await policy.GetLocationAsync();
                if (location is not null)
                    localLocations.Add(new(policy.Priority, location));
            }

            // always 1 location will match, if more, negotiate best location for instance
            var desiredLocation = localLocations.Count == 1
                ? localLocations.First().Item2
                : await NegotiateLocation(localLocations, member.AppliedPolicies);

            resultLocations.Add(desiredLocation);
        }

        // negotiate action-level location
        var locationsTmp = resultLocations.Select(x => new Tuple<Priority, PlannedLocation>(Priority.None, x))
            .ToHashSet();
        var negotiatedLocation = await NegotiateLocation(locationsTmp, allAppliedPolicies.ToList());
        return negotiatedLocation;
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
        var max = ordered.First().Item1;

        // less or equal to one because a location selected by policy will always match itself
        if (max <= 1) return ordered.First().Item3;

        // when no location can be selected, we return the default location
        var defaultLocation = _policyBuilder.GetDefaultLocation();
        return await defaultLocation.GetLocationAsync();
    }
}