using Middleware.Models.Enums;
using Middleware.ResourcePlanner.Models;

namespace Middleware.ResourcePlanner.Policies.LocationSelection;

internal class UrllcSliceLocation: ILocationSelectionPolicy
{
    /// <inheritdoc />
    public Priority Priority { get; }

    public UrllcSliceLocation(Priority priority)
    {
        Priority = priority;

    }
    /// <inheritdoc />
    public Task<Location> GetLocationAsync()
    {
        throw new NotImplementedException();
    }
}