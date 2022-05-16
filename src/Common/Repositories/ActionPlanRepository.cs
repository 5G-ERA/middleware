using Microsoft.Extensions.Logging;
using Middleware.Common.Enums;
using Middleware.Common.Models;
using RedisGraphDotNet.Client;
using StackExchange.Redis;

namespace Middleware.Common.Repositories;

public class ActionPlanRepository : BaseRepository<ActionPlanModel>, IActionPlanRepository
{
    /// <summary>
    /// Default constructor
    /// </summary>
    /// <param name="redisClient"></param>
    /// <param name="redisGraph"></param>
    /// <param name="logger"></param>
    public ActionPlanRepository(IConnectionMultiplexer redisClient, IRedisGraphClient redisGraph, ILogger<ActionPlanRepository> logger) : base(RedisDbIndexEnum.ActionSequence, redisClient, redisGraph, logger, false)
    {
    }
}