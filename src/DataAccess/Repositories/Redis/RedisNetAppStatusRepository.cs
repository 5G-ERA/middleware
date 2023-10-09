using Middleware.DataAccess.Repositories.Abstract;
using Middleware.Models.Domain;
using Middleware.Models.Dto;
using Neo4j.Driver;
using Redis.OM.Contracts;
using RedisGraphDotNet.Client;
using ILogger = Serilog.ILogger;

namespace Middleware.DataAccess.Repositories.Redis
{
    public class RedisNetAppStatusRepository : RedisRepository<NetAppStatusModel, NetAppStatusDto>, INetAppStatusRepository
    {
        public RedisNetAppStatusRepository (IRedisConnectionProvider provider, IRedisGraphClient redisGraph, Microsoft.Extensions.Logging.ILogger<RedisNetAppStatusRepository> logger, IDriver driver) : base(provider, redisGraph, true, logger, driver)
        {
        }

        public async Task<NetAppStatusModel> AddAsync(NetAppStatusModel model, Func<Guid> guidProvider)
        {
            model.Timestamp = DateTimeOffset.Now;
            return await base.AddAsync(model, guidProvider);
        }
    }
}
