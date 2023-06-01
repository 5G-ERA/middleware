namespace Middleware.Models.Domain.Contracts;

public interface IPolicyAssignable
{
    /// <summary>
    /// Lists the resource scoped policies applicable to the object
    /// </summary>
    List<string> AppliedPolicies { get; init; }
}