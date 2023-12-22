using Middleware.Models.Domain;
using Middleware.Models.Domain.Contracts;

namespace Middleware.ResourcePlanner.Policies;

internal interface ILocationSelectionPolicy : IPolicy
{
    /// <summary>
    ///     Did the location selection policy found matching location. False if the policy was not executed yet.
    /// </summary>
    bool FoundMatchingLocation { get; }

    /// <summary>
    ///     Evaluates the best possible plannedLocation based on the policy implementation
    /// </summary>
    /// <returns></returns>
    Task<PlannedLocation> GetLocationAsync(IHardwareRequirementClaim hwClaim = null);

    /// <summary>
    ///     Checks if the specified plannedLocation acceptable by the policy
    /// </summary>
    /// <param name="plannedLocation"></param>
    /// <returns></returns>
    Task<bool> IsLocationSatisfiedByPolicy(PlannedLocation plannedLocation);
}