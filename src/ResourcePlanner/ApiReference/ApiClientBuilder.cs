using Middleware.Common;
using Middleware.Common.Config;
using Middleware.ResourcePlanner.Orchestrator;

namespace Middleware.ResourcePlanner.ApiReference;

public class ApiClientBuilder : IApiClientBuilder
{
    private readonly IHttpClientFactory _httpClientFactory;
    private readonly IEnvironment _env;

    public ApiClientBuilder(IHttpClientFactory httpClientFactory, IEnvironment env)
    {
        _httpClientFactory = httpClientFactory;
        _env = env;
    }
    public OrchestratorApiClient CreateOrchestratorApiClient()
    {
        var address = _env.GetEnvVariable("ORCHESTRATOR_ADDRESS") ??
                      throw new ArgumentNullException("ORCHESTRATOR_ADDRESS", "ORCHESTRATOR_ADDRESS environment variable not specified");
        var client = _httpClientFactory.CreateClient(AppConfig.OrchestratorApiClientName);
        return new OrchestratorApiClient($"{address}", client);
    }
}