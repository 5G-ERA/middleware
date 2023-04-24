namespace Middleware.RedisInterface.Contracts.Responses;

public class GetPoliciesResponse
{
    public IEnumerable<PolicyResponse> Policies { get; set; } = Enumerable.Empty<PolicyResponse>();
}