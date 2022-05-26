using Microsoft.Extensions.Logging;
using Middleware.Common.Enums;
using Middleware.Common.Models;
using RedisGraphDotNet.Client;
using StackExchange.Redis;

namespace Middleware.Common.Repositories;

public class NetAppStatusRepository : BaseRepository<NetAppStatusModel>, INetAppStatusRepository
{
    public NetAppStatusRepository(RedisDbIndexEnum redisDbIndex, IConnectionMultiplexer redisClient, IRedisGraphClient redisGraph, ILogger<NetAppStatusRepository> logger) : base(redisDbIndex, redisClient, redisGraph, logger, false)
    {
    }
}