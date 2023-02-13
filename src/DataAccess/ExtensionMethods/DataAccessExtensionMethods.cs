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
    /// Configures the Redis Connection for the Application. The connection includes standard redis client via <see
    /// cref="ConnectionMultiplexer"/> and connection to Redis Graph using <seealso cref="RedisGraphClient"/>.
    /// </summary>
    /// <param name="builder"></param>
    /// <returns></returns>
    public static WebApplicationBuilder RegisterRedis(this WebApplicationBuilder builder)
    {
        var config = builder.Configuration.GetSection(RedisConfig.ConfigName).Get<RedisConfig>();

        ConnectionMultiplexer multiplexer = ConnectionMultiplexer.Connect(
            config.HostName,
            (c) =>
            {
                c.Password = config.Password;
            });
        builder.Services.AddSingleton<IConnectionMultiplexer>(multiplexer);
        RedisGraphClient redisGraphClient = new RedisGraphClient(multiplexer);
        builder.Services.AddSingleton<IRedisGraphClient>(redisGraphClient);
        
        var mux2 = ConnectionMultiplexer.Connect(config.ClusterHostname, c => c.Password = config.Password);
        IRedisConnectionProvider provider = new RedisConnectionProvider(mux2);
        builder.Services.AddSingleton(provider);
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

        return services;
    }
}
