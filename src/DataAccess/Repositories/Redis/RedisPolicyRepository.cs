using System.Collections.Generic;
using System.Text.Json;
using Microsoft.Extensions.Logging;
using Middleware.Common.Enums;
using Middleware.DataAccess.Repositories.Abstract;
using Middleware.Models.Domain;
using Middleware.Models.Dto;
using Middleware.Models.Enums;
using NReJSON;
using Redis.OM.Contracts;
using RedisGraphDotNet.Client;
using StackExchange.Redis;


namespace Middleware.DataAccess.Repositories.Redis
{
    public class RedisPolicyRepository : RedisRepository<PolicyModel, PolicyDto>, IPolicyRepository
    {
        /// <summary>
        /// Default constructor
        /// </summary>
        /// <param name="redisClient"></param>
        /// <param name="redisGraph"></param>
        /// <param name="logger"></param>

        public RedisPolicyRepository(IRedisConnectionProvider provider, IRedisGraphClient redisGraph) : base(provider, redisGraph, true)
        {
        }

        /// <summary>
        /// Check if a policy can coexist with already active policies
        /// </summary>
        /// <param name="newActivePolicy"></param>
        /// <param name="ActivePolicies"></param>
        /// <returns>boolean</returns>
        public bool CheckPolicyCanCoexist(PolicyModel newActivePolicy, List<PolicyModel> ActivePolicies)
        {
            foreach (PolicyModel policy in ActivePolicies)
            {
                if (policy.IsExclusiveWithinType == newActivePolicy.IsExclusiveWithinType)
                {
                    return false;
                }
            }
            return true;
        }

        /// <summary>
        /// Retrieves active policies
        /// </summary>
        /// <returns> Active policies </returns>
        public async Task<List<PolicyModel>> GetActivePoliciesAsync()
        {
            var activePolicies = FindQuery(Dto => Dto.IsActive == true).ToList().Select(x => ToTModel(x)).ToList();
            return activePolicies;
        }

        /// <summary>
        /// Patching properties for PolicyModel
        /// </summary>
        /// <param name="id"></param>
        /// <param name="patch"></param>
        /// <returns> Patched model </returns>
        public async Task<PolicyModel> PatchPolicyAsync(Guid id, PolicyModel patch)
        {
            PolicyModel? currentModel = await GetByIdAsync(id);
            if (currentModel == null)
            {
                return null;
            }
            if (!string.IsNullOrEmpty(patch.Name))
            {
                currentModel.Name = patch.Name;
            }
            if (!string.IsNullOrEmpty(patch.Timestamp.ToString()))
            {
                currentModel.Timestamp = patch.Timestamp;
            }
            if (!string.IsNullOrEmpty(patch.IsExclusiveWithinType.ToString()))
            {
                currentModel.IsExclusiveWithinType = patch.IsExclusiveWithinType;
            }
            if (patch.IsActive != null)
            {
                // Some policies cannot be active at the same time. Automatic check.
                if (patch.IsActive == true)
                {
                    List<PolicyModel> activePolicies = await GetActivePoliciesAsync();
                    bool coexistanceCheck = CheckPolicyCanCoexist(patch, activePolicies);
                    if (coexistanceCheck == true) { currentModel.IsActive = patch.IsActive; }
                    else { throw new Exception("The proposed policy cannot coexists with the already active policies."); }
                }
            }
            if (!string.IsNullOrEmpty(patch.Description))
            {
                currentModel.Description = patch.Description;
            }
            await UpdateAsync(currentModel);
            return currentModel;
        }
    }
}
