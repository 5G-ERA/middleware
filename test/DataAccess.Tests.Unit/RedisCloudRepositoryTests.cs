using FluentAssertions;
using Middleware.DataAccess.Repositories.Redis;
using Middleware.Models.Domain;
using NSubstitute;
using Redis.OM.Contracts;
using RedisGraphDotNet.Client;
using Serilog;
using StackExchange.Redis;

namespace DataAccess.Tests.Unit;

public class RedisCloudRepositoryTests
{
    private readonly RedisCloudRepository _sut;

    private readonly IRedisConnectionProvider _connectionProvider = Substitute.For<IRedisConnectionProvider>();
    private readonly IRedisGraphClient _graphClient = Substitute.For<IRedisGraphClient>();
    private readonly ILogger _logger = Substitute.For<ILogger>();

    public RedisCloudRepositoryTests()
    {
        _sut = new RedisCloudRepository(_connectionProvider, _graphClient, _logger);
    }

    [Fact]
    public async Task GetFreeCloudsIdsAsync_ShouldReturnFreeCloudsFromSpecifiedClouds_WhenSomeCloudsAreUnused()
    {
        // arrange
        var allClouds = GetExampleClouds();
        var unusedClouds = allClouds.Take(2).ToList();
        var occupiedClouds = allClouds.Skip(2).ToList();
        var guidStr = Guid.NewGuid().ToString();
        
        var dummyNode = new Node(1)
        {
            Properties = new Dictionary<string, RedisValue>()
            {
                { "ID", new RedisValue(guidStr) },
                { "Type", new RedisValue("INSTANCE") },
                { "Name", new RedisValue("Instance1") }
            }
        };
        
        var resultSet = new ResultSet();
        resultSet.Metrics = new QueryExecutionMetrics();
        resultSet.Results = new Dictionary<string, List<RedisGraphResult>>()
        {
            {
                "initiatesFrom", new List<RedisGraphResult>()
                {
                    dummyNode, dummyNode
                }
            },
            {
                "PointsTo", new List<RedisGraphResult>(occupiedClouds.Select((c, idx) => new Node(idx)
                {
                    Properties = new Dictionary<string, RedisValue>()
                    {
                        { "ID", new RedisValue(c.Id.ToString()) },
                        { "Type", new RedisValue("CLOUD") },
                        { "Name", new RedisValue(c.Name) }
                    }
                }))
            }
        };
        _graphClient.Query(Arg.Any<string>(), Arg.Any<string>()).Returns(resultSet);
        // act
        var result = await _sut.GetFreeCloudsIdsAsync(allClouds);
        // assert

        result.Should().BeEquivalentTo(unusedClouds);
    }

    private List<CloudModel> GetExampleClouds()
    {
        var retVal = new List<CloudModel>();
        for (int i = 0; i < 4; i++)
        {
            retVal.Add(new CloudModel()
            {
                Id = Guid.NewGuid(),
                Name = "Cloud" + (i + 1),
            });
        }

        return retVal;
    }
}