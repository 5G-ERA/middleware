using Middleware.Common;
using Middleware.TaskPlanner.Orchestrator;
using Middleware.TaskPlanner.ResourcePlanner;

namespace Middleware.TaskPlanner.ApiReference;

public class ApiClientBuilder : IApiClientBuilder
{
    private readonly IHttpClientFactory _httpClientFactory;
    private readonly IEnvironment _env;

    public ApiClientBuilder(IHttpClientFactory httpClientFactory, IEnvironment env)
    {
        _httpClientFactory = httpClientFactory;
        _env = env;
    }    

    /// <summary>
    /// <inheritdoc cref="IApiClientBuilder.CreateRedisApiClient"/>
    /// </summary>
    public ResourcePlannerApiClient CreateResourcePlannerApiClient()
    {
        var address = _env.GetEnvVariable("RESOURCE_PLANNER_API_SERVICE_HOST") ??
                      throw new ArgumentNullException("RESOURCE_PLANNER_API_SERVICE_HOST", "RESOURCE_PLANNER_API_SERVICE_HOST environment variable not specified");
        var url = $"http://{address}";
        var client = _httpClientFactory.CreateClient("resourcePlannerApiClient");
        return new ResourcePlannerApiClient(url, client);
    }
    
    /// <inheritdoc />
    public OrchestratorApiClient CreateOrchestratorApiClient()
    {
        var address = _env.GetEnvVariable("ORCHESTRATOR_API_SERVICE_HOST") ??
                      throw new ArgumentNullException("ORCHESTRATOR_API_SERVICE_HOST", "ORCHESTRATOR_API_SERVICE_HOST environment variable not specified");
        var url = $"http://{address}";
        var client = _httpClientFactory.CreateClient("orchestratorApiClient");
        return new OrchestratorApiClient(url, client);
    }
}

public interface IApiClientBuilder
{
    /// <summary>
    /// Creates the Resource planner api client
    /// </summary>
    /// <returns></returns>
    ResourcePlannerApiClient CreateResourcePlannerApiClient();

    /// <summary>
    /// Creates the Orchestrator client
    /// </summary>
    /// <returns></returns>
    OrchestratorApiClient CreateOrchestratorApiClient();
}