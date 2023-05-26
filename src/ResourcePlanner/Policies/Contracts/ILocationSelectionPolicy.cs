using Middleware.Models.Domain;

namespace Middleware.ResourcePlanner.Policies;

internal interface ILocationSelectionPolicy : IPolicy
{
    /// <summary>
    ///     Evaluates the best possible plannedLocation based on the policy implementation
    /// </summary>
    /// <returns></returns>
    Task<PlannedLocation> GetLocationAsync();

    /// <summary>
    ///     Checks if the specified plannedLocation acceptable by the policy
    /// </summary>
    /// <param name="plannedLocation"></param>
    /// <returns></returns>
    Task<bool> IsLocationSatisfiedByPolicy(PlannedLocation plannedLocation);
}