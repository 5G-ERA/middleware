using Middleware.Models.Domain.Contracts;
using Middleware.ResourcePlanner.Models;

namespace Middleware.ResourcePlanner.Policies;

internal interface IPolicyService
{
    Task<Location> GetLocationAsync(IReadOnlyList<IPolicyAssignable> members);
    Task ApplyPoliciesAsync(IPolicyAssignable member);
}