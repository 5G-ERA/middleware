using k8s.Models;
using Middleware.Common.Enums;
using Middleware.Common.ExtensionMethods;

namespace Middleware.Common.Config;

public static class AppConfig
{
    /// <summary>
    ///     Name of the system
    /// </summary>
    public const string SystemName = "Middleware";

    /// <summary>
    ///     Name of the <see cref="HttpClient" /> used to connect to RedisInterface API
    /// </summary>
    public const string RedisApiClientName = "redisApiClient";

    /// <summary>
    ///     Name of the <see cref="HttpClient" /> used to connect to RedisInterface API
    /// </summary>
    public const string OsmApiClientName = "osmApiClient";

    /// <summary>
    ///     Name of the <see cref="HttpClient" /> used to connect to Orchestrator API
    /// </summary>
    public const string OrchestratorApiClientName = "orchestratorApiClient";

    /// <summary>
    ///     Namespace in which the middleware pods will be deployed
    /// </summary>
    public const string K8SNamespaceName = "middleware";

    /// <summary>
    ///     Mapping of the services for the conversion of the YAML files
    /// </summary>
    public static readonly Dictionary<string, Type> K8STypeMappings = new()
        { { "v1/Pod", typeof(V1Pod) }, { "v1/Service", typeof(V1Service) }, { "apps/v1", typeof(V1Deployment) } };

    /// <summary>
    ///     Represents the Address under which the Middleware is accessible
    /// </summary>
    public static string MiddlewareAddress = string.Empty;

    /// <summary>
    ///     Interval in which the status check has to be performed. Expressed in seconds.
    /// </summary>
    public static readonly int StatusCheckInterval = 10;

    /// <summary>
    ///     Name of the network created by MultusCNI
    /// </summary>
    public static string MultusNetworkName = "ros-network-1";

    /// <summary>
    ///     Configuration of the application Development / Release
    /// </summary>
    public static string AppConfiguration { get; set; }

    public static string MiddlewareDeploymentLocationName { get; set; }

    public static Guid MiddlewareId { get; set; }

    /// <summary>
    ///     Is the application running in the Development environment
    /// </summary>
    /// <returns></returns>
    public static bool IsDevEnvironment()
    {
        return AppConfiguration == AppVersionEnum.Dev.GetStringValue();
    }

    public static string GetMiddlewareAddress()
    {
        var builder = new UriBuilder(MiddlewareAddress);
        builder.Path = "/status/netapp";
        return builder.ToString();
    }
}