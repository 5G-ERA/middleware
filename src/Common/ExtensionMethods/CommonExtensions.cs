using Amazon;
using Microsoft.AspNetCore.Builder;
using Microsoft.Extensions.Configuration;
using Microsoft.Extensions.DependencyInjection;
using Middleware.Common.Config;
using RedisGraphDotNet.Client;
using StackExchange.Redis;

namespace Middleware.Common.ExtensionMethods;

public static class CommonExtensions
{
    public static IServiceCollection RegisterCommonServices(this IServiceCollection services)
    {
        services.AddSingleton<IEnvironment, MiddlewareEnvironment>();
        return services;
    }
    /// <summary>
    /// Register the AWS Secrets Manager to retrieve the data for the appsettings.json file.
    /// </summary>
    /// <param name="builder"></param>
    /// <returns></returns>
    public static WebApplicationBuilder RegisterSecretsManager(this WebApplicationBuilder builder)
    {
        var awsKey = Environment.GetEnvironmentVariable("AWS_ACCESS_KEY_ID");
        var awsSecret = Environment.GetEnvironmentVariable("AWS_SECRET_ACCESS_KEY");

        if (string.IsNullOrWhiteSpace(awsKey) || string.IsNullOrWhiteSpace(awsSecret))
            return builder;

        builder.Configuration.AddSecretsManager(region: RegionEndpoint.EUWest1, configurator: opt =>
        {
            opt.SecretFilter = entry => entry.Name.StartsWith($"{AppConfig.SystemName}-");
            opt.KeyGenerator = (_, s) => s
                .Replace($"{AppConfig.SystemName}-", string.Empty)
                .Replace("__", ":");
        });
        builder.Services.Configure<ElasticConfig>(builder.Configuration.GetSection(ElasticConfig.ConfigName));
        return builder;
    }

    /// <summary>
    /// Configures the Redis Connection for the Application. The connection includes standard redis client via <see cref="ConnectionMultiplexer"/>
    /// and connection to Redis Graph using <seealso cref="RedisGraphClient"/>.
    /// </summary>
    /// <param name="builder"></param>
    /// <returns></returns>
    public static WebApplicationBuilder RegisterRedis(this WebApplicationBuilder builder) 
    {
        var config = builder.Configuration.GetSection(RedisConfig.ConfigName).Get<RedisConfig>();

        ConnectionMultiplexer multiplexer = ConnectionMultiplexer.Connect(config.HostName, (c) =>
        {
            c.Password = config.Password;
        });
        builder.Services.AddSingleton<IConnectionMultiplexer>(multiplexer);
        RedisGraphClient redisGraphClient = new RedisGraphClient(multiplexer);
        builder.Services.AddSingleton<IRedisGraphClient>(redisGraphClient);

        return builder;

    }
}