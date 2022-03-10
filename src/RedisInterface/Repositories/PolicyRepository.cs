using Middleware.Common.Models;
using RedisGraphDotNet.Client;
using StackExchange.Redis;

namespace Middleware.RedisInterface.Repositories
{
    public class PolicyRepository : BaseRepository<PolicyModel>, IPolicyRepository
    {
        public PolicyRepository(IConnectionMultiplexer redisClient, IRedisGraphClient redisGraph) : base(3, redisClient, redisGraph)
        {
        }
    }
}
