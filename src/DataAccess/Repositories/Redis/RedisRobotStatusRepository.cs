using Middleware.DataAccess.Repositories.Abstract;
using Middleware.Models.Domain;
using Middleware.Models.Dto;
using Redis.OM.Contracts;
using RedisGraphDotNet.Client;
using ILogger = Serilog.ILogger;

namespace Middleware.DataAccess.Repositories.Redis
{
    public class RedisRobotStatusRepository : RedisRepository<RobotStatusModel, RobotStatusDto>, IRobotStatusRepository
    {
        public RedisRobotStatusRepository(IRedisConnectionProvider provider, IRedisGraphClient redisGraph, ILogger logger) : base(provider, redisGraph, true, logger)
        {
        }

        public async Task<RobotStatusModel> AddAsync(RobotStatusModel model, Func<Guid> guidProvider)
        {
            model.Timestamp = DateTimeOffset.Now;
            return await base.AddAsync(model, guidProvider);
        }
    }
}
