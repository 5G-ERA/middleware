using Middleware.Models.Domain.Contracts;
using Middleware.ResourcePlanner.Models;

namespace Middleware.ResourcePlanner.Policies;

internal interface IPolicyService
{
    Location GetLocation(IReadOnlyList<IPolicyAssignable> members);
    void ApplyPolicies(IPolicyAssignable member);
}