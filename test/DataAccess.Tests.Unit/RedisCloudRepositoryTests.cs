using FluentAssertions;
using Middleware.DataAccess.Repositories;
using Middleware.Models.Domain;
using Neo4j.Driver;
using NSubstitute;
using Redis.OM.Contracts;
using RedisGraphDotNet.Client;
using Serilog;

namespace DataAccess.Tests.Unit;

//[LogTestExecution]
public class RedisCloudRepositoryTests
{
    private readonly IRedisConnectionProvider _connectionProvider = Substitute.For<IRedisConnectionProvider>();
    private readonly IRedisGraphClient _graphClient = Substitute.For<IRedisGraphClient>();
    private readonly Microsoft.Extensions.Logging.ILogger<RedisCloudRepository> _logger = Substitute.For<Microsoft.Extensions.Logging.ILogger<RedisCloudRepository>>();
    private readonly RedisCloudRepository _sut;
    private readonly IDriver _driver = Substitute.For<IDriver>();

    public RedisCloudRepositoryTests()
    {
        _sut = new(_connectionProvider, _graphClient, _logger, _driver);
    }

    [Fact(Skip ="Test in progress after changing graph provider")]
    public async Task GetFreeCloudsIdsAsync_ShouldReturnFreeCloudsFromSpecifiedClouds_WhenSomeCloudsAreUnused()
    {
        // arrange
        var allClouds = GetExampleClouds(4);
        var unusedClouds = allClouds.Take(2).ToList();
        var occupiedClouds = allClouds.Skip(2).ToList();
        var rng = new Random(420);
        var dummyNode = GetGraphNode();

        foreach (var dict in occupiedClouds)
        {
            var resultSet = new ResultSet();
            resultSet.Metrics = new();
            resultSet.Results = new()
            {
                {
                    "initiatesFrom", new(new[] { dummyNode })
                },
                {
                    "PointsTo", new()
                    {
                        new Node(rng.Next())
                        {
                            Properties = new()
                            {
                                { "ID", new(dict.Id.ToString()) },
                                { "Type", new("CLOUD") },
                                { "Name", new(dict.Name) }
                            }
                        }
                    }
                }
            };

            _graphClient.Query(Arg.Any<string>(), Arg.Is<string>(t => t.Contains(dict.Id.ToString())))
                .Returns(resultSet);
        }

        foreach (var cloud in unusedClouds)
        {
            var resultSet = new ResultSet();
            resultSet.Metrics = new();
            resultSet.Results = new();

            _graphClient.Query(Arg.Any<string>(), Arg.Is<string>(t => t.Contains(cloud.Id.ToString())))
                .Returns(resultSet);
        }

        // act
        var result = await _sut.GetFreeCloudsIdsAsync(allClouds);

        // assert
        result.Should().BeEquivalentTo(unusedClouds);
    }

    private static Node GetGraphNode()
    {
        var dummyNode = new Node(int.MaxValue)
        {
            Properties = new()
            {
                { "ID", new(Guid.NewGuid().ToString()) },
                { "Type", new("INSTANCE") },
                { "Name", new("Instance1") }
            }
        };
        return dummyNode;
    }

    [Fact(Skip = "Skipped because it is not needed for the latest release as semantic planning is disabled")]
    public async Task GetLessBusyCloudsAsync_ShouldReturnOrderedEdgesInAscendingOrder_WhenProvidedWithListOfClouds()
    {
        // arrange
        var rng = new Random(420);
        var allClouds = GetExampleClouds(20);
        var allCloudsWithCntDict = allClouds.ToDictionary(x => x, _ => rng.Next(0, 40));
        var dummyNode = GetGraphNode();

        foreach (var dict in allCloudsWithCntDict)
        {
            var resultSet = new ResultSet();
            resultSet.Metrics = new();
            resultSet.Results = new()
            {
                {
                    "initiatesFrom", new(Enumerable.Repeat(dummyNode, dict.Value))
                },
                {
                    "PointsTo", new(Enumerable.Repeat(new Node(rng.Next())
                    {
                        Properties = new()
                        {
                            { "ID", new(dict.Key.Id.ToString()) },
                            { "Type", new("CLOUD") },
                            { "Name", new(dict.Key.Name) }
                        }
                    }, dict.Value))
                }
            };

            _graphClient.Query(Arg.Any<string>(), Arg.Is<string>(t => t.Contains(dict.Key.Id.ToString())))
                .Returns(resultSet);
        }

        var sortedDict = allCloudsWithCntDict.OrderBy(x => x.Value).ToDictionary(x => x.Key, pair => pair.Value);
        var expectedList = sortedDict.Select(x => x.Key).ToList();
        // act
        var result = await _sut.GetLessBusyCloudsAsync(allClouds);

        // assert
        result.Should().BeEquivalentTo(expectedList);
        result.Should().ContainInOrder(expectedList);
        result.Should().HaveSameCount(allClouds);
    }

    private List<CloudModel> GetExampleClouds(int cnt)
    {
        var retVal = new List<CloudModel>();
        for (var i = 0; i < cnt; i++)
        {
            retVal.Add(new()
            {
                Id = Guid.NewGuid(),
                Name = "Cloud" + (i + 1)
            });
        }

        return retVal;
    }
}