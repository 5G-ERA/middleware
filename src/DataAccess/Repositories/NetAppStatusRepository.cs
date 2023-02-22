using Microsoft.Extensions.Logging;
using Middleware.Common.Enums;
using Middleware.DataAccess.Repositories.Abstract;
using Middleware.Models.Domain;
using Middleware.Models.Enums;
using RedisGraphDotNet.Client;
using StackExchange.Redis;

namespace Middleware.DataAccess.Repositories;

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