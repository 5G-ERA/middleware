using Middleware.ResourcePlanner.Policies.LocationSelection;

namespace Middleware.ResourcePlanner.Policies;

internal interface IPolicyBuilder
{
    /// <summary>
    ///     Gets the implementation for the PlannedLocation policy
    /// </summary>
    /// <param name="policyName"></param>
    /// <returns>Implementation of the PlannedLocation Selection Policy or null when specified policy is of different type</returns>
    Task<ILocationSelectionPolicy> CreateLocationPolicy(string policyName);

    /// <summary>
    ///     Build <see cref="DefaultLocation" /> policy
    /// </summary>
    /// <returns></returns>
    DefaultLocation GetDefaultLocationPolicy();
}