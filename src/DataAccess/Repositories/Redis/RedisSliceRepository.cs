using Middleware.DataAccess.Repositories.Abstract;
using Middleware.Models.Domain.Slice;
using Middleware.Models.Dto.Slice;
using Neo4j.Driver;
using Redis.OM.Contracts;
using RedisGraphDotNet.Client;
using Serilog;

namespace Middleware.DataAccess.Repositories;

public class RedisSliceRepository : RedisRepository<SliceModel, SliceDto>, ISliceRepository
{
    public RedisSliceRepository(IRedisConnectionProvider provider, IRedisGraphClient redisGraph, Microsoft.Extensions.Logging.ILogger<RedisSliceRepository> logger, IDriver driver) : base(
        provider, redisGraph, true, logger, driver)
    {
    }
}