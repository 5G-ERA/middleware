using Microsoft.AspNetCore.Builder;
using Microsoft.Extensions.Configuration;
using Microsoft.Extensions.DependencyInjection;
using Middleware.Common.Config;
using Middleware.DataAccess.Repositories;
using Middleware.DataAccess.Repositories.Abstract;
using Middleware.DataAccess.Repositories.Redis;
using Redis.OM;
using Redis.OM.Contracts;
using RedisGraphDotNet.Client;
using StackExchange.Redis;

namespace Middleware.DataAccess.ExtensionMethods;

public static class DataAccessExtensionMethods
{
    /// <summary>
    ///     Configures the Redis Connection for the Application. The connection includes standard redis client via
    ///     <see
    ///         cref="ConnectionMultiplexer" />
    ///     and connection to Redis Graph using <seealso cref="RedisGraphClient" />.
    /// </summary>
    /// <param name="builder"></param>
    /// <returns></returns>
    public static WebApplicationBuilder RegisterRedis(this WebApplicationBuilder builder)
    {
        var config = builder.Configuration.GetSection(RedisConfig.ConfigName).Get<RedisConfig>();

        //For the redis-cluster master node, port 6380 should be used, using port 6379 will point to the replicas of the redis-cluster
        var mux = ConnectionMultiplexer.Connect($"{config.ClusterHostname}:6380", c => c.Password = config.Password);
        IRedisConnectionProvider provider = new RedisConnectionProvider(mux);
        builder.Services.AddSingleton(provider);
        builder.Services.AddSingleton<IConnectionMultiplexer>(mux);
        var redisGraphClient = new RedisGraphClient(mux);
        builder.Services.AddSingleton<IRedisGraphClient>(redisGraphClient);
        return builder;
    }

    public static IServiceCollection RegisterRepositories(this IServiceCollection services)
    {
        services.AddScoped<IActionRepository, RedisActionRepository>();
        services.AddScoped<IActionPlanRepository, RedisActionPlanRepository>();
        services.AddScoped<ICloudRepository, RedisCloudRepository>();
        services.AddScoped<IContainerImageRepository, RedisContainerImageRepository>();
        services.AddScoped<IEdgeRepository, RedisEdgeRepository>();
        services.AddScoped<IInstanceRepository, RedisInstanceRepository>();
        services.AddScoped<IPolicyRepository, RedisPolicyRepository>();
        services.AddScoped<IRobotRepository, RedisRobotRepository>();
        services.AddScoped<ITaskRepository, RedisTaskRepository>();
        services.AddScoped<IUserRepository, RedisUserRepository>();
        services.AddScoped<ISliceRepository, RedisSliceRepository>();

        return services;
    }
}