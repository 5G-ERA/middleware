﻿using Microsoft.Extensions.Logging;
using Middleware.Common.Enums;
using Middleware.Common.Models;
using RedisGraphDotNet.Client;
using StackExchange.Redis;

namespace Middleware.Common.Repositories;

public class NetAppStatusRepository : BaseRepository<NetAppStatusModel>, INetAppStatusRepository
{
    public NetAppStatusRepository(IConnectionMultiplexer redisClient, IRedisGraphClient redisGraph, ILogger<NetAppStatusRepository> logger) : base(RedisDbIndexEnum.NetAppStatus, redisClient, redisGraph, logger, false)
    {
    }
}