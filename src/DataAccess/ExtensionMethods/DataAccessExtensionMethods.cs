using InfluxDB.Client;
using Microsoft.AspNetCore.Builder;
using Microsoft.Extensions.Configuration;
using Microsoft.Extensions.DependencyInjection;
using Middleware.Common.Config;
using Middleware.DataAccess.Repositories;
using Middleware.DataAccess.Repositories.Abstract;
using Middleware.DataAccess.Repositories.Abstract.Influx;
using Middleware.DataAccess.Repositories.Influx;
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
        var mux = ConnectionMultiplexer.Connect($"{config.ClusterHostname}:6379", c => c.Password = config.Password);
        IRedisConnectionProvider provider = new RedisConnectionProvider(mux);
        builder.Services.AddSingleton(provider);
        builder.Services.AddSingleton<IConnectionMultiplexer>(mux);
        var redisGraphClient = new RedisGraphClient(mux);
        builder.Services.AddSingleton<IRedisGraphClient>(redisGraphClient);
        return builder;
    }

    public static WebApplicationBuilder RegisterInflux(this WebApplicationBuilder builder)
    {
        var config = builder.Configuration.GetSection(InfluxConfig.ConfigName).Get<InfluxConfig>();
        var influxClient = new InfluxDBClient(config.Address, config.ApiKey);
        builder.Services.AddSingleton<IInfluxDBClient>(influxClient);

        builder.Services.AddScoped<IInfluxNetAppStatusRepository, InfluxNetAppStatusRepository>();
        builder.Services.AddScoped<IInfluxRobotStatusRepository, InfluxRobotStatusRepository>();
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
        services.AddScoped<ISystemConfigRepository, SystemConfigRepository>();
        services.AddScoped<ILocationRepository, RedisLocationRepository>();

        return services;
    }

    public static IServiceCollection RegisterCentralApiRepositories(this IServiceCollection services)
    {
        services.AddScoped<ICloudRepository, RedisCloudRepository>();
        services.AddScoped<IEdgeRepository, RedisEdgeRepository>();
        services.AddScoped<IRobotRepository, RedisRobotRepository>();
        services.AddScoped<ILocationRepository, RedisLocationRepository>();
        services.AddScoped<ISystemConfigRepository, SystemConfigRepository>();
        return services;
    }
}