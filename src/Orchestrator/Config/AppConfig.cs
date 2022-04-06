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
}