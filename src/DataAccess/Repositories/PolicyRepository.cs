using System.Text.Json;
using Microsoft.Extensions.Logging;
using Middleware.DataAccess.Repositories.Abstract;
using Middleware.Models.Domain;
using Middleware.Models.Enums;
using NReJSON;
using RedisGraphDotNet.Client;
using StackExchange.Redis;

namespace Middleware.DataAccess.Repositories
{
    public class PolicyRepository : BaseRepository<PolicyModel>, IPolicyRepository
    {
        /// <summary>
        /// Default constructor
        /// </summary>
        /// <param name="redisClient"></param>
        /// <param name="redisGraph"></param>
        /// <param name="logger"></param>
        public PolicyRepository(IConnectionMultiplexer redisClient, IRedisGraphClient redisGraph, ILogger<PolicyRepository> logger) : base(RedisDbIndexEnum.Policy, redisClient, redisGraph, logger, false)
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
        /// Retrieve all Policy models
        /// </summary>
        /// <returns> The list of policies </returns>
        public async Task<List<PolicyModel>> GetAllPoliciesAsync()
        {
            var keys = await GetKeysAsync("GetKeys");
            var policies = new List<PolicyModel>();

            foreach (var key in keys)
            {
                string val = (string)await Db.JsonGetAsync(key);
                PolicyModel policy = JsonSerializer.Deserialize<PolicyModel>(val);
                policies.Add(policy);
            }
            return policies;
        }

        /// <summary>
        /// Retrieves active policies
        /// </summary>
        /// <returns> Active policies </returns>
        public async Task<List<PolicyModel>> GetActivePoliciesAsync()
        {
            List<PolicyModel> activePolicies = await ExecuteLuaQueryAsync("GetActivePolicies");

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
            string model = (string)await Db.JsonGetAsync(id.ToString());
            PolicyModel currentModel = JsonSerializer.Deserialize<PolicyModel>(model);
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
            await Db.JsonSetAsync(id.ToString(), JsonSerializer.Serialize(currentModel));
            return currentModel;
        }
    }
}
