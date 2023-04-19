using Microsoft.Extensions.DependencyInjection;
using Middleware.Common.Exceptions;
using Middleware.RedisInterface.Sdk.Client;
using Refit;

namespace Middleware.RedisInterface.Sdk;

public static class ServiceCollectionExtensions
{
    /// <summary>
    /// Adds RedisInterface client tht connects internally to the RedisInterface in the k8s cluster
    /// </summary>
    /// <param name="services"></param>
    /// <returns></returns>
    public static IServiceCollection AddRedisInterfaceClient(this IServiceCollection services)
    {
        var hostname = Environment.GetEnvironmentVariable("REDIS_INTERFACE_API_SERVICE_HOST");

        if (hostname is null)
        {
            throw new MiddlewareConfigurationException("REDIS_INTERFACE_API_SERVICE_HOST environment variable is not set");
        }

        services.AddRefitClient<IRedisInterface>()
            .ConfigureHttpClient(c => c.BaseAddress = new Uri($"http://{hostname}"));
        services.AddScoped<IRedisInterfaceClient, RedisInterfaceClient>();
        return services;
    }
}