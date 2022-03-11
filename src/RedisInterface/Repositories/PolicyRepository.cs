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
        public PolicyRepository(IConnectionMultiplexer redisClient, IRedisGraphClient redisGraph) : base(RedisDbIndexEnum.Policies, redisClient, redisGraph)
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
    }
}
