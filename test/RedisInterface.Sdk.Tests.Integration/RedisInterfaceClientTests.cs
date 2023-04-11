using Microsoft.AspNetCore.Mvc.Testing;
using Microsoft.Extensions.Logging;
using Middleware.RedisInterface;
using Middleware.RedisInterface.Sdk;
using Middleware.RedisInterface.Sdk.Client;
using NSubstitute;
using Refit;

namespace RedisInterface.Sdk.Tests.Integration;

public class RedisInterfaceClientTests : IClassFixture<WebApplicationFactory<IApiMarker>>
{
    private readonly ILogger<RedisInterfaceClient> _logger = Substitute.For<ILogger<RedisInterfaceClient>>();
    private IRedisInterfaceClient _sut;
    
    public RedisInterfaceClientTests(WebApplicationFactory<IApiMarker> appFactory)
    {
        var httpClient = appFactory.CreateClient();
        var api = RestService.For<IRedisInterface>(httpClient);
        _sut = new RedisInterfaceClient(api, _logger);
    }
    
    [Fact]
    public async Task GetCloudByNameAsync_ReturnsCloud_WhenCloudWithNameIsFound()
    {
        
    }
}