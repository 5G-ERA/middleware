using Middleware.Common.MessageContracts;
using Middleware.Models.ExtensionMethods;
using Yarp.ReverseProxy.Configuration;
using Yarp.ReverseProxy.Transforms;

namespace Middleware.OcelotGateway.Services;

public class GatewayConfigurationService
{
    private readonly InMemoryConfigProvider _inMemoryConfigProvider;
    private readonly ILogger<GatewayConfigurationService> _logger;

    public GatewayConfigurationService(IProxyConfigProvider inMemoryConfigProvider,
        ILogger<GatewayConfigurationService> logger)
    {
        _logger = logger;
        if (inMemoryConfigProvider is InMemoryConfigProvider imcp)
            _inMemoryConfigProvider = imcp;
    }

    public void CreateDynamicRoute(GatewayAddNetAppEntryMessage msg)
    {
        var config = _inMemoryConfigProvider.GetConfig();

        var clusterList = config.Clusters.ToList();
        var routeList = config.Routes.ToList();

        var clusterCfg = new ClusterConfig
        {
            ClusterId = msg.NetAppName + "-Cluster",
            Destinations = new Dictionary<string, DestinationConfig>
            {
                { "destination1", new DestinationConfig { Address = $"http://{msg.NetAppName}" } }
            }
        };
        var path = msg.Route.SanitizeToUriPath();
        _logger.LogInformation("Opening new route with path: {path}", path);
        var routeCfg = new RouteConfig
        {
            RouteId = msg.NetAppName + "-Route",
            Match = new()
            {
                Path = path + "/{**remainder}"
            },
            ClusterId = clusterCfg.ClusterId //
        };
        // transforms allow us to change the path that is requested like below to replace direct forwarding
        routeCfg = routeCfg.WithTransformPathRemovePrefix($"/{msg.NetAppName}");

        clusterList.Add(clusterCfg);
        routeList.Add(routeCfg);

        _inMemoryConfigProvider.Update(routeList, clusterList);

        _logger.LogInformation("Finished updating path to the new NetApp");
    }

    public void DeleteDynamicRoute(GatewayDeleteNetAppEntryMessage msg)
    {
        var config = _inMemoryConfigProvider.GetConfig();

        var clusterList = config.Clusters.ToList();
        var routeList = config.Routes.ToList();

        var matchRouteToDelete = msg.NetAppName + "-Route";
        RouteConfig routeToDelete = null;
        foreach (var route in routeList)
        {
            if (route.RouteId == matchRouteToDelete) routeToDelete = route;
        }

        routeList.Remove(routeToDelete);

        var matchClusterToDelete = msg.NetAppName + "-Cluster";
        ClusterConfig clusterToDelete = null;
        foreach (var cluster in clusterList)
        {
            if (cluster.ClusterId == matchClusterToDelete) clusterToDelete = cluster;
        }

        clusterList.Remove(clusterToDelete);

        _inMemoryConfigProvider.Update(routeList, clusterList);
    }
}