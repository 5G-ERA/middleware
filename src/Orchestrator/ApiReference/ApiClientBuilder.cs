using Middleware.Common;
using Middleware.Orchestrator.Config;
using Middleware.Orchestrator.Osm;
using Middleware.Orchestrator.RedisInterface;

namespace Middleware.Orchestrator.ApiReference;

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
    public RedisApiClient CreateRedisApiClient()
    {
        var address = _env.GetEnvVariable("REDIS_INTERFACE_ADDRESS") ??
                      throw new ArgumentNullException("REDIS_INTERFACE_ADDRESS", "REDIS_INTERFACE_ADDRESS environment variable not specified");
        var client = _httpClientFactory.CreateClient(AppConfig.RedisApiClientName);
        return new RedisApiClient($"{address}", client);
    }
    
    /// <summary>
    /// <inheritdoc cref="IApiClientBuilder.CreateOsmApiClient"/>
    /// </summary>
    public OsmClient CreateOsmApiClient()
    {
        var client = _httpClientFactory.CreateClient(AppConfig.OsmApiClientName);
        client.BaseAddress = new Uri("https://osm.api:9999");
        return new OsmClient(client);
    }
}