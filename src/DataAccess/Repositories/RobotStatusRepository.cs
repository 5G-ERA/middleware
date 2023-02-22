using Microsoft.Extensions.Logging;
using Middleware.Common.Enums;
using Middleware.DataAccess.Repositories.Abstract;
using Middleware.Models.Domain;
using Middleware.Models.Enums;
using RedisGraphDotNet.Client;
using StackExchange.Redis;

namespace Middleware.DataAccess.Repositories;

public class RobotStatusRepository : BaseRepository<RobotStatusModel>, IRobotStatusRepository
{
    /// <summary>
    /// Default constructor
    /// </summary>
    /// <param name="redisClient"></param>
    /// <param name="redisGraph"></param>
    /// <param name="logger"></param>
    public RobotStatusRepository(IConnectionMultiplexer redisClient, IRedisGraphClient redisGraph, ILogger<RobotStatusRepository> logger)
        : base(RedisDbIndexEnum.RobotStatus, redisClient, redisGraph, logger, false)
    {
    }
    ///<inheritdoc/>
    public override async Task<RobotStatusModel> AddAsync(RobotStatusModel model, Func<Guid> guidProvider)
    {
        model.Timestamp = DateTimeOffset.Now;
        return await base.AddAsync(model, guidProvider);
    }
}