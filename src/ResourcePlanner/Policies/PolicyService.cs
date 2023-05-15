using Middleware.Models.Domain.Contracts;
using Middleware.RedisInterface.Sdk;

namespace Middleware.ResourcePlanner.Policies;

internal class PolicyService : IPolicyService
{
    private readonly IRedisInterfaceClient _redisInterfaceClient;

    public PolicyService(IRedisInterfaceClient redisInterfaceClient)
    {
        _redisInterfaceClient = redisInterfaceClient;
    }
    /// <inheritdoc />
    public void ApplyPolicies(IPolicyAssignable member)
    {
        throw new NotImplementedException();
    }
}