using Middleware.ResourcePlanner.Models;

namespace Middleware.ResourcePlanner.Policies;

internal interface ILocationSelectionPolicy : IPolicy
{
    /// <summary>
    ///     Evaluates the best possible location based on the policy implementation
    /// </summary>
    /// <returns></returns>
    Task<Location> GetLocationAsync();

    /// <summary>
    ///     Checks if the specified location acceptable by the policy
    /// </summary>
    /// <param name="location"></param>
    /// <returns></returns>
    Task<bool> IsLocationSatisfiedByPolicy(Location location);
}