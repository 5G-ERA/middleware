using Middleware.Orchestrator.Osm;
using Middleware.Orchestrator.RedisInterface;

namespace Middleware.Orchestrator.ApiReference;

public interface IApiClientBuilder
{
    /// <summary>
    /// Builds the ready to use instance of the <see cref="RedisApiClient"/>
    /// </summary>
    /// <returns></returns>
    RedisApiClient CreateRedisApiClient();
    /// <summary>
    /// Builds the ready to use instance of the <see cref="OsmClient"/>
    /// </summary>
    /// <returns></returns>
    OsmClient CreateOsmApiClient();
}