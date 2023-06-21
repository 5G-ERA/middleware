using Microsoft.AspNetCore.Mvc;
using Yarp.ReverseProxy.Configuration;

namespace Middleware.OcelotGateway.Controllers;

[Route("api/v1/[controller]")]
[ApiController]
public class RouteController : ControllerBase
{
    private readonly InMemoryConfigProvider _inMemoryConfigProvider;

    public string RouteId { get; set; }

    public string ClusterId { get; set; }


    public RouteController(IProxyConfigProvider inMemoryConfigProvider)
    {
        if (inMemoryConfigProvider is InMemoryConfigProvider imcp)
            _inMemoryConfigProvider = imcp;
    }


    [HttpGet]
    [Route("hello", Name = "Hello")]
    public IActionResult Hello()
    {
        return Ok("hello");
    }

    [HttpPost]
    [Route("configure", Name = "CreateDynamicRoute")]
    public IActionResult CreateDynamicRoute()
    {
        var config = _inMemoryConfigProvider.GetConfig();

        var clusterList = config.Clusters.ToList();
        var routeList = config.Routes.ToList();

        var clusterCfg = new ClusterConfig
        {
            ClusterId = "testCluster",
            Destinations = new Dictionary<string, DestinationConfig>
            {
                { "testdest1", new DestinationConfig { Address = "http://taskplanner.api/api/v1/test" } }
            }
        };
        var routecfg = new RouteConfig
        {
            RouteId = "test",
            Match = new()
            {
                Path = "hello"
            },
            ClusterId = "testCluster" //
        };
        clusterList.Add(clusterCfg);
        routeList.Add(routecfg);

        _inMemoryConfigProvider.Update(routeList, clusterList);

        return Ok();
    }
}