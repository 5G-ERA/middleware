using Middleware.Common;
using Middleware.Common.Config;
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
        var address = _env.GetEnvVariable("REDIS_INTERFACE_API_SERVICE_HOST") ??
                      throw new ArgumentNullException("REDIS_INTERFACE_API_SERVICE_HOST", "REDIS_INTERFACE_API_SERVICE_HOST environment variable not specified");
        var url = $"http://{address}";
        var client = _httpClientFactory.CreateClient(AppConfig.RedisApiClientName);
        return new RedisApiClient(url, client);
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