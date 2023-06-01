using Bogus;
using Microsoft.Extensions.Logging;
using Middleware.RedisInterface.Contracts.Requests;
using Middleware.RedisInterface.Sdk;
using Middleware.RedisInterface.Sdk.Client;
using NSubstitute;
using Refit;

namespace RedisInterface.Sdk.Tests.Integration;

//[LogTestExecution]
public class RedisInterfaceClientTests : IClassFixture<RedisInterfaceApiFactory>
{
    private readonly Faker<CloudRequest> _cloudFaker = new Faker<CloudRequest>()
        .RuleFor(cloud => cloud.Name, faker => faker.Company.CompanyName().Replace(" ", string.Empty))
        .RuleFor(cloud => cloud.Status, "Online")
        .RuleFor(cloud => cloud.Type, "Cloud")
        .RuleFor(cloud => cloud.Cpu, 2)
        .RuleFor(cloud => cloud.MacAddress, faker => faker.System.Version().ToString())
        .RuleFor(cloud => cloud.IpAddress, faker => new(faker.Internet.Ip(), UriKind.RelativeOrAbsolute));

    private readonly HttpClient _httpClient;
    private readonly ILogger<RedisInterfaceClient> _logger = Substitute.For<ILogger<RedisInterfaceClient>>();
    private IRedisInterfaceClient _sut;

    public RedisInterfaceClientTests(RedisInterfaceApiFactory appFactory)
    {
        _httpClient = appFactory.CreateClient();
        var api = RestService.For<IRedisInterface>(_httpClient);
        _sut = new RedisInterfaceClient(api, _logger);
    }

    [Fact(Skip = "Test is not yet ready")]
    public async Task GetCloudByNameAsync_ReturnsCloud_WhenCloudWithNameIsFound()
    {
        // var cloudRequest = _cloudFaker.Generate();
        // var createdResponse = await _httpClient.PostAsJsonAsync("api/v1/Cloud", cloudRequest);
        // var createdCloud = await createdResponse.Content.ReadFromJsonAsync<CloudResponse>();
        //
        // var response = await _sut.CloudGetByNameAsync(cloudRequest.Name);
        //
        // response.Should().BeEquivalentTo(createdCloud);
    }
}