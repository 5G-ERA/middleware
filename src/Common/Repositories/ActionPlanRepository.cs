using Microsoft.Extensions.Logging;
using Middleware.Common.Enums;
using Middleware.Common.Models;
using RedisGraphDotNet.Client;
using StackExchange.Redis;

namespace Middleware.Common.Repositories;

public class ActionPlanRepository : BaseRepository<ActionPlanModel>, IActionPlanRepository
{
    public ActionPlanRepository(IConnectionMultiplexer redisClient, IRedisGraphClient redisGraph, ILogger logger) : base(RedisDbIndexEnum.ActionSequence, redisClient, redisGraph, logger, false)
    {
    }
}