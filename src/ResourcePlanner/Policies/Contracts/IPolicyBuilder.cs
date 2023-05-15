using Middleware.Models.Enums;

namespace Middleware.ResourcePlanner.Policies;

internal interface IPolicyBuilder
{
    /// <summary>
    /// Gets the implementation for the Location policy
    /// </summary>
    /// <param name="policyName"></param>
    /// <returns></returns>
    ILocationSelectionPolicy GetLocationPolicy(string policyName);
}