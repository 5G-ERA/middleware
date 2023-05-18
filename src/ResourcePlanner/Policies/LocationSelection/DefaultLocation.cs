using Microsoft.Extensions.Options;
using Middleware.Common.Config;
using Middleware.Models.Enums;
using Middleware.ResourcePlanner.Models;

namespace Middleware.ResourcePlanner.Policies.LocationSelection;

/// <summary>
///     Represents the default Location Selection Policy.
///     By design this policy always returns current <see cref="Location" /> and all <see cref="Location" /> selected by
///     other policies satisfy this Policy.
/// </summary>
internal class DefaultLocation : ILocationSelectionPolicy
{
    private readonly IOptions<MiddlewareConfig> _middlewareOptions;

    public DefaultLocation(IOptions<MiddlewareConfig> middlewareOptions)
    {
        _middlewareOptions = middlewareOptions;
    }

    /// <inheritdoc />
    public Priority Priority => Priority.Low;

    /// <inheritdoc />
    public Task<Location> GetLocationAsync()
    {
        var location = new Location
        {
            Type = _middlewareOptions.Value.InstanceType,
            Name = _middlewareOptions.Value.InstanceName
        };
        return Task.FromResult(location);
    }

    /// <summary>
    ///     Checks if the specified location acceptable by the policy.
    ///     By default, <see cref="DefaultLocation" /> is satisfied with Locations selected by other policies.
    /// </summary>
    /// <param name="location"></param>
    /// <returns></returns>
    public Task<bool> IsLocationSatisfiedByPolicy(Location location)
    {
        return Task.FromResult(true);
    }
}