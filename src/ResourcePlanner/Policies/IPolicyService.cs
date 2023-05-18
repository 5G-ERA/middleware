using Middleware.Models.Domain.Contracts;
using Middleware.ResourcePlanner.Models;

namespace Middleware.ResourcePlanner.Policies;

internal interface IPolicyService
{
    /// <summary>
    ///     Calculates the location based on the assigned policies
    /// </summary>
    /// <param name="members"></param>
    /// <returns></returns>
    Task<Location> GetLocationAsync(IReadOnlyList<IPolicyAssignable> members);
}