namespace Middleware.Models.Domain.Contracts;

public interface IPolicyAssignable
{
    List<string> AppliedPolicies { get; init; }
}