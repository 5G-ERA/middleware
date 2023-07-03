using Middleware.Common.MessageContracts;
using Yarp.ReverseProxy.Configuration;
using Yarp.ReverseProxy.Transforms;

namespace Middleware.OcelotGateway.Services;

public class GatewayConfigurationService
{
    private readonly InMemoryConfigProvider _inMemoryConfigProvider;

    public GatewayConfigurationService(IProxyConfigProvider inMemoryConfigProvider)
    {
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
        var routeCfg = new RouteConfig
        {
            RouteId = msg.NetAppName + "-Route",
            Match = new()
            {
                Path = msg.Route
            },
            ClusterId = clusterCfg.ClusterId //
        };
        // transforms allow us to change the path that is requested like below to replace direct forwarding
        routeCfg = routeCfg.WithTransformPathRemovePrefix($"/{msg.NetAppName}");

        clusterList.Add(clusterCfg);
        routeList.Add(routeCfg);

        _inMemoryConfigProvider.Update(routeList, clusterList);
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