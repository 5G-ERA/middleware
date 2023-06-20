using Microsoft.AspNetCore.Builder;
using Microsoft.AspNetCore.Mvc;
using Yarp.ReverseProxy.Configuration;
using Microsoft.AspNetCore.Routing;


namespace Middleware.OcelotGateway.Controllers;


[Route("api/v1")]
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
        
        ClusterConfig clusterCfg = new ClusterConfig
        {
            ClusterId = "testCluster",
            Destinations = new Dictionary<string, DestinationConfig> 
            { 
                { "testdest1", new DestinationConfig{Address = "http://localhost/api/v1/hello"} }
            }
        };
        RouteConfig routecfg = new RouteConfig()
        {
            RouteId = "test",
            Match = new RouteMatch
            {
                Path = "my/test/endpoint"
            }
        };
        clusterList.Add(clusterCfg);
        routeList.Add(routecfg);

        _inMemoryConfigProvider.Update(routeList, clusterList);

        return Ok();
    }

}
