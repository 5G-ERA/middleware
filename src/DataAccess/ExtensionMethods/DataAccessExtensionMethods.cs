using Microsoft.AspNetCore.Builder;
using Microsoft.Extensions.Configuration;
using Microsoft.Extensions.DependencyInjection;
using Middleware.Common.Config;
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

        return builder;
    }
}
