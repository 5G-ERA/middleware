using Middleware.RedisInterface.Sdk;

namespace Middleware.ResourcePlanner.Policies;

internal class PolicyBuilder : IPolicyBuilder
{
    private readonly IRedisInterfaceClient _redisInterfaceClient;

    public PolicyBuilder(IRedisInterfaceClient redisInterfaceClient)
    {
        _redisInterfaceClient = redisInterfaceClient;
    }
    public ILocationSelectionPolicy GetLocationPolicy(string policyName)
    {
        var policy = _redisInterfaceClient.GetPolicyByNameAsync(policyName);
    }
}