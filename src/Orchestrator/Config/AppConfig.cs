using System.Security.Policy;
using k8s.Models;

namespace Middleware.Orchestrator.Config;

public static class AppConfig
{
    /// <summary>
    /// Name of the <see cref="HttpClient"/> used to connect to RedisInterface API
    /// </summary>
    public const string RedisApiClientName = "redisApiClient";
    /// <summary>
    /// Name of the <see cref="HttpClient"/> used to connect to RedisInterface API
    /// </summary>
    public const string OsmApiClientName = "osmApiClient";
    /// <summary>
    /// Namespace in which the middleware pods will be deployed
    /// </summary>
    public static string K8SNamespaceName { get; set; } = "middleware";
    /// <summary>
    /// Mapping of the services for the conversion of the YAML files
    /// </summary>
    public static readonly Dictionary<string, Type> K8STypeMappings = new()
        {{"v1/Pod", typeof(V1Pod)}, {"v1/Service", typeof(V1Service)}, {"apps/v1", typeof(V1Deployment)}};
    /// <summary>
    /// Configuration of the application Development / Release
    /// </summary>
    public static string AppConfiguration { get; set; }
}