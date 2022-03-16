using Middleware.Common.Models;
using Middleware.RedisInterface.Enums;
using Middleware.RedisInterface.Repositories.Abstract;
using RedisGraphDotNet.Client;
using StackExchange.Redis;

namespace Middleware.RedisInterface.Repositories
{
    public class ActionRepository : BaseRepository<ActionModel>, IActionRepository
    {
        public ActionRepository(IConnectionMultiplexer redisClient, IRedisGraphClient redisGraph) : base(RedisDbIndexEnum.Actions, redisClient, redisGraph)
        {
        }


        public Task<ActionModel> PatchActionAsync(Guid id)
        {
            throw new NotImplementedException();
        }
    }
}
