using Microsoft.Extensions.Options;
using Middleware.Common.Config;
using Middleware.Models.Domain.Contracts;
using Middleware.Models.Enums;
using Middleware.RedisInterface.Sdk;
using Middleware.ResourcePlanner.Models;
using Middleware.ResourcePlanner.Policies.LocationSelection;

namespace Middleware.ResourcePlanner.Policies;

internal class PolicyService : IPolicyService
{
    private readonly IRedisInterfaceClient _redisInterfaceClient;
    private readonly IPolicyBuilder _policyBuilder;
    private readonly IOptions<MiddlewareConfig> _middlewareConfig;


    public PolicyService(IRedisInterfaceClient redisInterfaceClient, IPolicyBuilder policyBuilder, IOptions<MiddlewareConfig> middlewareConfig)
    {
        _redisInterfaceClient = redisInterfaceClient;
        _policyBuilder = policyBuilder;
        _middlewareConfig = middlewareConfig;
    }

    /// <inheritdoc />
    public async Task<Location> GetLocationAsync(IReadOnlyList<IPolicyAssignable> members)
    {
        var locations = new Dictionary<Priority, Location>();
        foreach (var member in members)
        {
            if (member.AppliedPolicies.Any() == false)
            {
                var policy = new DefaultLocation(_middlewareConfig);
                var location = await policy.GetLocationAsync();
                locations.Add(policy.Priority, location);
                continue;
            }
            foreach (var policyName in member.AppliedPolicies)
            {
                var policy = await _policyBuilder.CreateLocationPolicy(policyName);
                if (policy is null)
                    continue;

                var location = await policy.GetLocationAsync();

                locations.Add(policy.Priority, location);
            }
        }
        var retVal = locations.MaxBy(t => (int)t.Key);
        return retVal.Value;
    }

    /// <inheritdoc />
    public Task ApplyPoliciesAsync(IPolicyAssignable member)
    {
        throw new NotImplementedException();
    }
}