using Middleware.DataAccess.Repositories.Abstract;
using Middleware.Models.Domain;
using Middleware.Models.Dto;
using Redis.OM.Contracts;
using RedisGraphDotNet.Client;
using Serilog;

namespace Middleware.DataAccess.Repositories;

public class SystemConfigRepository : RedisRepository<SystemConfigModel, SystemConfigDto>, ISystemConfigRepository
{
    /// <inheritdoc />
    public SystemConfigRepository(IRedisConnectionProvider provider, IRedisGraphClient redisGraph,
        bool isWritableToGraph, ILogger logger) : base(provider, redisGraph, isWritableToGraph, logger)
    {
    }

    /// <inheritdoc />
    public async Task<SystemConfigModel?> GetConfigAsync()
    {
        var cfg = await GetByIdAsync(Guid.Empty);

        return cfg;
    }
}