using System.Text;
using k8s.Models;
using Middleware.Common.Enums;
using Middleware.Common.ExtensionMethods;

namespace Middleware.Common.Config;

public static class AppConfig
{
    /// <summary>
    /// Name of the system
    /// </summary>
    public const string SystemName = "Middleware";
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
    /// <summary>
    /// Is the application running in the Development environment
    /// </summary>
    /// <returns></returns>
    public static bool IsDevEnvironment() => AppConfiguration == AppVersionEnum.Dev.GetStringValue();
    /// <summary>
    /// Represents the Address under which the Middleware is accessible
    /// </summary>
    public static string MiddlewareAddress = string.Empty;
    /// <summary>
    /// Interval in which the status check has to be performed. Expressed in seconds.
    /// </summary>
    public static int StatusCheckInterval = 10;

    public static string GetMiddlewareAddress()
    {
        var builder = new UriBuilder(MiddlewareAddress);
        builder.Path = "/status/netapp";
        return builder.ToString();
    }
}