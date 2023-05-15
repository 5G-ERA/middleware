using Microsoft.Extensions.Options;
using Middleware.Common.Config;
using Middleware.Models.Enums;
using Middleware.ResourcePlanner.Models;

namespace Middleware.ResourcePlanner.Policies.LocationSelection;

internal class DefaultLocation : ILocationSelectionPolicy
{
    private readonly IOptions<MiddlewareConfig> _middlewareOptions;

    /// <inheritdoc />
    public Priority Priority { get; } = Priority.Low;

    public DefaultLocation(IOptions<MiddlewareConfig> middlewareOptions)
    {
        _middlewareOptions = middlewareOptions;
    }
    /// <inheritdoc />
    public async Task<Location> GetLocationAsync()
    {
        var location = new Location()
        {
            Type = _middlewareOptions.Value.InstanceType,
            Name = _middlewareOptions.Value.InstanceName
        };
        return await Task.FromResult(location);
    }
}