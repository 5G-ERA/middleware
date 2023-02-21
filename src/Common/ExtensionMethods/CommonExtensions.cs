using System.Net.Http.Headers;
using Amazon;
using Microsoft.AspNetCore.Builder;
using Microsoft.AspNetCore.Http;
using Microsoft.Extensions.Configuration;
using Microsoft.Extensions.DependencyInjection;
using Middleware.Common.Config;
using Middleware.Common.Services;
using RedisGraphDotNet.Client;
using StackExchange.Redis;

namespace Middleware.Common.ExtensionMethods;

public static class CommonExtensions
{
    public static IServiceCollection RegisterCommonServices(this IServiceCollection services)
    {
        services.AddSingleton<IEnvironment, MiddlewareEnvironment>();
        
        services.AddHttpClient(AppConfig.RedisApiClientName, (a) =>
        {
            a.BaseAddress = new Uri(Environment.GetEnvironmentVariable("REDIS_INTERFACE_ADDRESS"));
            a.DefaultRequestHeaders.Accept.Add(
                new MediaTypeWithQualityHeaderValue("application/json"));
        });
        services.AddScoped<IRedisInterfaceClientService, RedisInterfaceClientService>();
        
        
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
        builder.Services.Configure<CustomLoggerConfig>(builder.Configuration.GetSection(CustomLoggerConfig.ConfigName));
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

    public static IServiceCollection AddUriHelper(this IServiceCollection services)
    {

        services.AddHttpContextAccessor();
        services.AddSingleton<IUriService>(o =>
        {
            var accessor = o.GetRequiredService<IHttpContextAccessor>();
            var request = accessor.HttpContext.Request;
            var uri = string.Concat(request.Scheme, "://", request.Host.ToUriComponent());
            return new UriService(uri);
        });
        return services;
    }
    
}