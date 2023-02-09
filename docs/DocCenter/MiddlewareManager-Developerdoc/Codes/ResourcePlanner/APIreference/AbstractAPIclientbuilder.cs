using Middleware.ResourcePlanner.Orchestrator;
using Middleware.ResourcePlanner.RedisInterface;

namespace Middleware.ResourcePlanner.ApiReference;

public interface IApiClientBuilder
{
    /// <summary>
    /// Builds the ready to use instance of the <see cref="RedisApiClient"/>
    /// </summary>
    /// <returns></returns>
    RedisApiClient CreateRedisApiClient();
    /// <summary>
    /// Builds the ready to use instance of the <see cref="OrchestratorApiClient"/>
    /// </summary>
    /// <returns></returns>
    OrchestratorApiClient CreateOrchestratorApiClient();
}