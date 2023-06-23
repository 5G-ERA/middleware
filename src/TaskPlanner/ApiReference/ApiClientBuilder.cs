using Middleware.Common;
using Middleware.TaskPlanner.ResourcePlanner;

namespace Middleware.TaskPlanner.ApiReference;

public class ApiClientBuilder : IApiClientBuilder
{
    private readonly IEnvironment _env;
    private readonly IHttpClientFactory _httpClientFactory;

    public ApiClientBuilder(IHttpClientFactory httpClientFactory, IEnvironment env)
    {
        _httpClientFactory = httpClientFactory;
        _env = env;
    }

    /// <summary>
    ///     <inheritdoc cref="IApiClientBuilder.CreateRedisApiClient" />
    /// </summary>
    public ResourcePlannerApiClient CreateResourcePlannerApiClient()
    {
        var address = _env.GetEnvVariable("RESOURCE_PLANNER_API_SERVICE_HOST") ??
                      throw new ArgumentNullException("RESOURCE_PLANNER_API_SERVICE_HOST",
                          "RESOURCE_PLANNER_API_SERVICE_HOST environment variable not specified");
        var url = $"http://{address}";
        var client = _httpClientFactory.CreateClient("resourcePlannerApiClient");
        return new(url, client);
    }
}

public interface IApiClientBuilder
{
    /// <summary>
    ///     Creates the Resource planner api client
    /// </summary>
    /// <returns></returns>
    ResourcePlannerApiClient CreateResourcePlannerApiClient();
}