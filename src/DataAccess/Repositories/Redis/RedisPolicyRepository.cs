using Middleware.DataAccess.Repositories.Abstract;
using Middleware.Models.Domain;
using Middleware.Models.Dto;
using Middleware.Models.Enums;
using Redis.OM.Contracts;
using RedisGraphDotNet.Client;
using Serilog;

namespace Middleware.DataAccess.Repositories;

public class RedisPolicyRepository : RedisRepository<PolicyModel, PolicyDto>, IPolicyRepository
{
    /// <summary>
    ///     Default constructor
    /// </summary>
    /// <param name="provider"></param>
    /// <param name="redisGraph"></param>
    /// <param name="logger"></param>
    public RedisPolicyRepository(IRedisConnectionProvider provider, IRedisGraphClient redisGraph, ILogger logger) :
        base(provider, redisGraph, false, logger)
    {
    }

    /// <summary>
    ///     Retrieves active policies
    /// </summary>
    /// <returns> Active policies </returns>
    public async Task<List<PolicyModel>> GetActivePoliciesAsync()
    {
        var activePolicies =
            await Task.Run(() => FindQuery(dto => dto.IsActive == true).ToList().Select(ToTModel).ToList());
        return activePolicies;
    }

    /// <inheritdoc />
    public async Task<PolicyModel?> GetPolicyByName(string name)
    {
        return await FindSingleAsync(p => p.Name == name);
    }

    /// <summary>
    ///     Patching properties for PolicyModel
    /// </summary>
    /// <param name="id"></param>
    /// <param name="patch"></param>
    /// <returns> Patched model </returns>
    public async Task<PolicyModel?> PatchPolicyAsync(Guid id, PolicyModel patch)
    {
        var currentModel = await GetByIdAsync(id);
        if (currentModel == null) return null;
        if (!string.IsNullOrEmpty(patch.Name)) currentModel.Name = patch.Name;
        if (patch.Type != PolicyType.None) currentModel.Type = patch.Type;
        if (patch.Timestamp != default) currentModel.Timestamp = patch.Timestamp;

        // Some policies cannot be active at the same time. Automatic check.
        if (patch.IsActive)
        {
            var activePolicies = await GetActivePoliciesAsync();
            var coexistenceCheck = CheckPolicyCanCoexist(patch, activePolicies);
            if (coexistenceCheck)
                currentModel.IsActive = patch.IsActive;
            else
                throw new("The proposed policy cannot coexists with the already active policies.");
        }

        if (!string.IsNullOrEmpty(patch.Description)) currentModel.Description = patch.Description;
        if (!string.IsNullOrEmpty(patch.IsExclusiveWithinType.ToString()))
            currentModel.IsExclusiveWithinType = patch.IsExclusiveWithinType;
        await UpdateAsync(currentModel);
        return currentModel;
    }

    /// <inheritdoc />
    public async Task<IReadOnlyCollection<PolicyModel>> GetActiveSystemPoliciesAsync()
    {
        var policies = await FindAsync(p => p.IsActive && p.Scope == PolicyScope.System.ToString());
        return policies.AsReadOnly();
    }

    /// <summary>
    ///     Check if a policy can coexist with already active policies
    /// </summary>
    /// <param name="newActivePolicy"></param>
    /// <param name="activePolicies"></param>
    /// <returns>Can exclusivity be applied (bool)</returns>
    private static bool CheckPolicyCanCoexist(PolicyModel newActivePolicy, List<PolicyModel> activePolicies)
    {
        foreach (var policy in activePolicies)
        {
            if (newActivePolicy.Type == policy.Type &&
                policy.IsExclusiveWithinType == newActivePolicy.IsExclusiveWithinType) return false;
        }

        return true;
    }
}