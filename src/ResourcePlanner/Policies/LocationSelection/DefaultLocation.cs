using Microsoft.Extensions.Options;
using Middleware.Common.Config;
using Middleware.Models.Domain;
using Middleware.Models.Enums;
using Middleware.RedisInterface.Sdk;

namespace Middleware.ResourcePlanner.Policies.LocationSelection;

/// <summary>
///     Represents the default PlannedLocation Selection Policy.
///     By design this policy always returns current <see cref="PlannedLocation" /> and all <see cref="PlannedLocation" />
///     selected by
///     other policies satisfy this Policy.
/// </summary>
internal class DefaultLocation : ILocationSelectionPolicy
{
    private readonly IOptions<MiddlewareConfig> _middlewareOptions;
    private readonly IRedisInterfaceClient _redisInterfaceClient;

    public DefaultLocation(IOptions<MiddlewareConfig> middlewareOptions, IRedisInterfaceClient redisInterfaceClient)
    {
        _middlewareOptions = middlewareOptions;
        _redisInterfaceClient = redisInterfaceClient;
    }

    /// <inheritdoc />
    public Priority Priority => Priority.Low;

    /// <inheritdoc />
    public bool FoundMatchingLocation { get; private set; }

    /// <inheritdoc />
    public async Task<PlannedLocation> GetLocationAsync()
    {
        Guid id;
        if (_middlewareOptions.Value.InstanceType == LocationType.Cloud.ToString())
        {
            var cloud = await _redisInterfaceClient.GetCloudByNameAsync(_middlewareOptions.Value.InstanceName);
            id = cloud!.Id;
        }
        else
        {
            var edgeResponse = await _redisInterfaceClient.GetEdgeByNameAsync(_middlewareOptions.Value.InstanceName);
            id = edgeResponse!.Id;
        }

        var location = new PlannedLocation(id, _middlewareOptions.Value.InstanceName,
            Enum.Parse<LocationType>(_middlewareOptions.Value.InstanceType));

        FoundMatchingLocation = true;

        return location;
    }

    /// <summary>
    ///     Checks if the specified plannedLocation acceptable by the policy.
    ///     By default, <see cref="DefaultLocation" /> is satisfied with Locations selected by other policies.
    /// </summary>
    /// <param name="plannedLocation"></param>
    /// <returns></returns>
    public Task<bool> IsLocationSatisfiedByPolicy(PlannedLocation plannedLocation)
    {
        return Task.FromResult(true);
    }
}