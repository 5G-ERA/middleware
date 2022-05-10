using System.Text.Json;
using Middleware.Common.Models;
using Middleware.Common.Enums;
using NReJSON;
using RedisGraphDotNet.Client;
using StackExchange.Redis;
using Microsoft.Extensions.Logging;

namespace Middleware.Common.Repositories
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
            if (!string.IsNullOrEmpty(patch.Timestamp.ToString()))
            {
                currentModel.Timestamp = patch.Timestamp;
            }
            if (patch.IsActive != null)
            {
                currentModel.IsActive = patch.IsActive;
            }
            if (!string.IsNullOrEmpty(patch.Description))
            {
                currentModel.Description = patch.Description;
            }
            if (!string.IsNullOrEmpty(patch.Name))
            {
                currentModel.Name = patch.Name;
            }
            await Db.JsonSetAsync(id.ToString(), JsonSerializer.Serialize(currentModel));
            return currentModel;
        }
    }
}
