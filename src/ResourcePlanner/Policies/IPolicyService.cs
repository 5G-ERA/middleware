using Middleware.Models.Domain.Contracts;

namespace Middleware.ResourcePlanner.Policies;

internal interface IPolicyService
{
    void ApplyPolicies(IPolicyAssignable member);
}