using System.Text.Json;
using Middleware.Common.Models;
using Middleware.RedisInterface.Enums;
using NReJSON;
using RedisGraphDotNet.Client;
using StackExchange.Redis;

namespace Middleware.RedisInterface.Repositories
{
    public class PolicyRepository : BaseRepository<PolicyModel>, IPolicyRepository
    {
        public PolicyRepository(IConnectionMultiplexer redisClient, IRedisGraphClient redisGraph, ILogger<PolicyRepository> logger) : base(RedisDbIndexEnum.Policy, redisClient, redisGraph, logger)
        {
        }

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

        public async Task<List<PolicyModel>> GetActivePoliciesAsync()
        {
            List<PolicyModel> activePolicies = await ExecuteLuaQueryAsync("GetActivePolicies");

            return activePolicies;
        }

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
            if (!string.IsNullOrEmpty(patch.PolicyName))
            {
                currentModel.PolicyName = patch.PolicyName;
            }
            await Db.JsonSetAsync(id.ToString(), JsonSerializer.Serialize(currentModel));
            return currentModel;
        }
    }
}
