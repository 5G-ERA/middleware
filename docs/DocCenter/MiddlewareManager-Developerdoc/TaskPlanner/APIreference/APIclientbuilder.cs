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
        var address = _env.GetEnvVariable("RESOURCE_PLANNER_ADDRESS") ??
                      throw new ArgumentNullException("RESOURCE_PLANNER_ADDRESS", "RESOURCE_PLANNER_ADDRESS environment variable not specified");
        var client = _httpClientFactory.CreateClient("resourcePlannerApiClient");
        return new ResourcePlannerApiClient(address, client);
    }
    
    /// <inheritdoc />
    public OrchestratorApiClient CreateOrchestratorApiClient()
    {
        var address = _env.GetEnvVariable("ORCHESTRATOR_ADDRESS") ??
                      throw new ArgumentNullException("ORCHESTRATOR_ADDRESS", "ORCHESTRATOR_ADDRESS environment variable not specified");
        var client = _httpClientFactory.CreateClient("orchestratorApiClient");
        return new OrchestratorApiClient(address, client);
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