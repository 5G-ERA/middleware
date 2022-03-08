using Middleware.Common.Models;
using RedisGraphDotNet.Client;
using StackExchange.Redis;

namespace Middleware.RedisInterface.Repositories
{
    public class PolicyRepository : BaseRepository<PolicyModel>, IPolicyRepository
    {
        public PolicyRepository(int redisDbIndex, IConnectionMultiplexer redisClient, IRedisGraphClient redisGraph) : base(redisDbIndex, redisClient, redisGraph)
        {
        }
    }
}
