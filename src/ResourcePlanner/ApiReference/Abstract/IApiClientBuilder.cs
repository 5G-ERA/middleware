using Middleware.ResourcePlanner.Orchestrator;

namespace Middleware.ResourcePlanner.ApiReference;

public interface IApiClientBuilder
{
    /// <summary>
    /// Builds the ready to use instance of the <see cref="OrchestratorApiClient"/>
    /// </summary>
    /// <returns></returns>
    OrchestratorApiClient CreateOrchestratorApiClient();
}