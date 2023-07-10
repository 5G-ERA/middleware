using Microsoft.Extensions.Options;
using Middleware.CentralApi.Sdk;
using Middleware.Common.Config;
using Middleware.Models.Enums;
using Middleware.RedisInterface.Contracts.Mappings;
using Middleware.RedisInterface.Sdk;
using Middleware.ResourcePlanner.Policies.LocationSelection;

namespace Middleware.ResourcePlanner.Policies;

internal class PolicyBuilder : IPolicyBuilder
{
    private readonly Dictionary<string, IPolicy> _cachedPolicies = new();
    private readonly ICentralApiClient _centralApi;
    private readonly ILogger _logger;
    private readonly IOptions<MiddlewareConfig> _middlewareConfig;
    private readonly IRedisInterfaceClient _redisInterfaceClient;

    public PolicyBuilder(IRedisInterfaceClient redisInterfaceClient, IOptions<MiddlewareConfig> middlewareConfig,
        ICentralApiClient centralApi, ILogger<PolicyBuilder> logger)
    {
        _redisInterfaceClient = redisInterfaceClient;
        _middlewareConfig = middlewareConfig;
        _centralApi = centralApi;
        _logger = logger;
    }

    public async Task<ILocationSelectionPolicy> CreateLocationPolicy(string policyName)
    {
        if (string.IsNullOrWhiteSpace(policyName))
            throw new ArgumentException("Value cannot be null or whitespace.", nameof(policyName));

        if (_cachedPolicies.TryGetValue(policyName, out var cachedPolicy))
            return (ILocationSelectionPolicy)cachedPolicy;

        var policyResp = await _redisInterfaceClient.GetPolicyByNameAsync(policyName);
        if (policyResp is null) return null;

        var policy = policyResp.ToPolicy();

        if (policy.IsActive == false) return null;
        if (policy.Type != PolicyType.LocationSelection) return null;

        ILocationSelectionPolicy policyImplementation = policyName switch
        {
            nameof(UrllcSliceLocation) => new UrllcSliceLocation(policy.Priority, _redisInterfaceClient, _centralApi,
                _logger),
            _ => new DefaultLocation(_middlewareConfig, _redisInterfaceClient)
        };
        _cachedPolicies[policyName] = policyImplementation;
        return policyImplementation;
    }

    /// <inheritdoc />
    public DefaultLocation GetDefaultLocationPolicy()
    {
        return new(_middlewareConfig, _redisInterfaceClient);
    }
}