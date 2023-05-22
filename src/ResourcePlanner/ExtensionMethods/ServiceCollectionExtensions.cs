using Middleware.Common.Config;
using Middleware.Common.Exceptions;
using Middleware.RedisInterface.Sdk;
using Middleware.ResourcePlanner.SliceManager;
using Refit;

namespace Middleware.ResourcePlanner.ExtensionMethods;

internal static class ServiceCollectionExtensions
{
    /// <summary>
    ///     Adds RedisInterface client tht connects internally to the RedisInterface in the k8s cluster
    /// </summary>
    /// <param name="services"></param>
    /// <param name="config"></param>
    /// <returns></returns>
    public static IServiceCollection AddSliceManagerClient(this IServiceCollection services, SliceConfig config)
    {
        var hostname = config.Hostname;

        if (hostname is null)
            throw new MiddlewareConfigurationException(
                "Slice Manager hostname is not set");

        services.AddRefitClient<ISliceManagerApi>()
            .ConfigureHttpClient(c => c.BaseAddress = new(hostname));
        services.AddScoped<IRedisInterfaceClient, RedisInterfaceClient>();
        return services;
    }
}