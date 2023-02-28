namespace Middleware.RedisInterface.Contracts.Responses;

public class GetAllPoliciesResponse
{
    public IEnumerable<PolicyResponse> Policies { get; set; } = Enumerable.Empty<PolicyResponse>();
}