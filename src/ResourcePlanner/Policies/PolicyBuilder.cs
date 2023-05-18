using Microsoft.Extensions.Options;
using Middleware.Common.Config;
using Middleware.Models.Enums;
using Middleware.RedisInterface.Contracts.Mappings;
using Middleware.RedisInterface.Sdk;
using Middleware.ResourcePlanner.Policies.LocationSelection;

namespace Middleware.ResourcePlanner.Policies;

internal class PolicyBuilder : IPolicyBuilder
{
    private readonly IOptions<MiddlewareConfig> _middlewareConfig;
    private readonly IRedisInterfaceClient _redisInterfaceClient;

    public PolicyBuilder(IRedisInterfaceClient redisInterfaceClient, IOptions<MiddlewareConfig> middlewareConfig)
    {
        _redisInterfaceClient = redisInterfaceClient;
        _middlewareConfig = middlewareConfig;
    }

    public async Task<ILocationSelectionPolicy?> CreateLocationPolicy(string policyName)
    {
        var policyResp = await _redisInterfaceClient.GetPolicyByNameAsync(policyName);
        var policy = policyResp.ToPolicy();

        if (policy.Type != PolicyType.LocationSelection) return null;

        ILocationSelectionPolicy policyImplementation = policyName switch
        {
            nameof(UrllcSliceLocation) => new UrllcSliceLocation(policy.Priority),
            _ => new DefaultLocation(_middlewareConfig)
        };

        return policyImplementation;
    }

    /// <inheritdoc />
    public DefaultLocation GetDefaultLocation()
    {
        return new(_middlewareConfig);
    }
}