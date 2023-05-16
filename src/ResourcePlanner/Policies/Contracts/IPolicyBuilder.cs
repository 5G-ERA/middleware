using Middleware.Models.Enums;

namespace Middleware.ResourcePlanner.Policies;

internal interface IPolicyBuilder
{
    /// <summary>
    /// Gets the implementation for the Location policy
    /// </summary>
    /// <param name="policyName"></param>
    /// <returns>Implementation of the Location Selection Policy or null when specified policy is of different type</returns>
    Task<ILocationSelectionPolicy> CreateLocationPolicy(string policyName);
}