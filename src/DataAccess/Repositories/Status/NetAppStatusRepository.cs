using System.Reflection.Metadata.Ecma335;
using DataAccess.Repositories.Abstract;
using Microsoft.Extensions.Logging;
using Middleware.Common.Enums;
using Middleware.Common.Models;
using RedisGraphDotNet.Client;
using StackExchange.Redis;

namespace DataAccess.Repositories.Status;

public class NetAppStatusRepository : BaseRepository<NetAppStatusModel>, INetAppStatusRepository
{
    public NetAppStatusRepository(IConnectionMultiplexer redisClient, IRedisGraphClient redisGraph, ILogger<NetAppStatusRepository> logger) : base(RedisDbIndexEnum.NetAppStatus, redisClient, redisGraph, logger, false)
    {
    }

    ///<inheritdoc/>
    public override async Task<NetAppStatusModel> AddAsync(NetAppStatusModel model, Func<Guid> guidProvider)
    {
        model.Timestamp = DateTimeOffset.Now;
        return await base.AddAsync(model, guidProvider);
    }
}