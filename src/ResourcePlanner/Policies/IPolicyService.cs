using Middleware.Models.Domain;

namespace Middleware.ResourcePlanner.Policies;

internal interface IPolicyService
{
    /// <summary>
    ///     Calculates the location based on the assigned policies
    /// </summary>
    /// <param name="members"></param>
    /// <returns></returns>
    Task<PlannedLocation> GetLocationAsync(IReadOnlyList<InstanceModel> members);
}