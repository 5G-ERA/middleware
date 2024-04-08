using System.Data.SqlTypes;
using System.Security.Policy;
using System.Text;
using MassTransit.Configuration;
using Microsoft.AspNetCore;
using Microsoft.Extensions.Options;
using Middleware.Common.Config;
using Middleware.Common.MessageContracts;
using Middleware.Models.ExtensionMethods;
using Yarp.ReverseProxy.Configuration;
using Yarp.ReverseProxy.Transforms;

namespace Middleware.OcelotGateway.Services;

public class GatewayConfigurationService
{
    private readonly InMemoryConfigProvider _inMemoryConfigProvider;
    private readonly ILogger<GatewayConfigurationService> _logger;
    private readonly IOptions<MiddlewareConfig> _mwConfig;

    public GatewayConfigurationService(IProxyConfigProvider inMemoryConfigProvider,
        ILogger<GatewayConfigurationService> logger, IOptions<MiddlewareConfig> mwConfig)
    {
        _logger = logger;
        _mwConfig = mwConfig;
        if (inMemoryConfigProvider is InMemoryConfigProvider imcp)
            _inMemoryConfigProvider = imcp;
    }

    public void CreateDynamicRoute(GatewayAddNetAppEntryMessage msg)
    {
        var config = _inMemoryConfigProvider.GetConfig();

        var clusterList = config.Clusters.ToList();
        var routeList = config.Routes.ToList();

        var address = $"http://{msg.NetAppName}.middleware.svc.cluster.local";
        var clusterCfg = new ClusterConfig
        {
            ClusterId = msg.NetAppName + "-Cluster",
            Destinations = new Dictionary<string, DestinationConfig>
            {
                {
                    "destination1",
                    new DestinationConfig { Address = address }
                }
            }
        };
        _logger.LogInformation("{netAppName} started adding new route with address: {address}", msg.NetAppName,
            address);
        var path = msg.Route.SanitizeToUriPath();
        _logger.LogInformation("Opening new route with path: {path}", path);
        var sanitized = msg.NetAppName.SanitizeToUriPath();
        var mwAddress = new Uri(_mwConfig.Value.Address);
        var protocol = mwAddress.IsAbsoluteUri ? mwAddress.Scheme : "http";
        var domain = mwAddress.IsAbsoluteUri ? mwAddress.Host : mwAddress.ToString();
        int port = mwAddress.IsAbsoluteUri ? mwAddress.Port : 80;

        string modifiedAddress = $"{protocol}://{sanitized}.{domain}:{port}";

        Uri uri = new Uri(modifiedAddress);
        var validHost = uri.Host;

        var routeCfg = new RouteConfig
        {
            RouteId = msg.NetAppName + "-Route",
            Match = new()
            {                
                Hosts = new[] { validHost }            
            },
            ClusterId = clusterCfg.ClusterId //
        };
        var routeSocketIoCfg = new RouteConfig
        {
            RouteId = msg.NetAppName + "SocketIO-Route",
            Match = new()
            {
                Path = "/socket.io/{**remainder}"
            },
            ClusterId = clusterCfg.ClusterId
        };

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
        var socketIoRouteName = msg.NetAppName + "SocketIO-Route";
        RouteConfig routeToDelete = null;
        RouteConfig socketIoRoute = null;
        foreach (var route in routeList)
        {
            if (route.RouteId == matchRouteToDelete) routeToDelete = route;
            if (route.RouteId == socketIoRouteName) socketIoRoute = route;
        }

        routeList.Remove(routeToDelete);
        routeList.Remove(socketIoRoute);

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