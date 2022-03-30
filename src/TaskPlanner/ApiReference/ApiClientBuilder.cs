using Middleware.TaskPlanner.Config;
using Middleware.TaskPlanner.RedisInterface;
using Middleware.TaskPlanner.ResourcePlanner;

namespace Middleware.TaskPlanner.ApiReference;

public class ApiClientBuilder : IApiClientBuilder
{
    private readonly IHttpClientFactory _httpClientFactory;

    public ApiClientBuilder(IHttpClientFactory httpClientFactory)
    {
        _httpClientFactory = httpClientFactory;
    }

    /// <summary>
    /// <inheritdoc cref="IApiClientBuilder.CreateRedisApiClient"/>
    /// </summary>
    public RedisApiClient CreateRedisApiClient()
    {
        var address = Environment.GetEnvironmentVariable("REDIS_INTERFACE_ADDRESS") ??
                      throw new ArgumentNullException("REDIS_INTERFACE_ADDRESS", "REDIS_INTERFACE_ADDRESS environment variable not specified");
        var client = _httpClientFactory.CreateClient("redisApiClient");
        return new RedisApiClient(address, client);
    }

    /// <summary>
    /// <inheritdoc cref="IApiClientBuilder.CreateRedisApiClient"/>
    /// </summary>
    public ResourcePlannerApiClient CreateResourcePlannerApiClient()
    {
        var address = Environment.GetEnvironmentVariable("RESOURCE_PLANNER_ADDRESS") ??
                      throw new ArgumentNullException("RESOURCE_PLANNER_ADDRESS", "RESOURCE_PLANNER_ADDRESS environment variable not specified");
        var client = _httpClientFactory.CreateClient("resourcePlannerApiClient");
        return new ResourcePlannerApiClient(address, client);
    }
}

public interface IApiClientBuilder
{
    RedisApiClient CreateRedisApiClient();
    ResourcePlannerApiClient CreateResourcePlannerApiClient();
}