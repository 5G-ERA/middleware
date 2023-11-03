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
public class RedisEdgeRepositoryTests
{
    private readonly IRedisConnectionProvider _connectionProvider = Substitute.For<IRedisConnectionProvider>();
    private readonly IRedisGraphClient _graphClient = Substitute.For<IRedisGraphClient>();
    private readonly Microsoft.Extensions.Logging.ILogger<RedisEdgeRepository> _logger = Substitute.For<Microsoft.Extensions.Logging.ILogger<RedisEdgeRepository>>();
    private readonly RedisEdgeRepository _sut;
    private readonly IDriver _driver = Substitute.For<IDriver>();

    public RedisEdgeRepositoryTests()
    {
        _sut = new(_connectionProvider, _graphClient, _logger, _driver);
    }

    [Fact(Skip = "Test in progress after changing graph provider")]
    public async Task GetFreeEdgesIdsAsync_ShouldReturnFreeEdgesFromSpecifiedEdges_WhenSomeEdgesAreUnused()
    {
        // arrange
        var allEdges = GetExampleEdges();
        var unusedEdges = allEdges.Take(2).ToList();
        var occupiedEdges = allEdges.Skip(2).ToList();
        var guidStr = Guid.NewGuid().ToString();
        var rng = new Random(420);
        var dummyNode = new Node(int.MaxValue)
        {
            Properties = new()
            {
                { "ID", new(guidStr) },
                { "Type", new("INSTANCE") },
                { "Name", new("Instance1") }
            }
        };

        foreach (var dict in occupiedEdges)
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
                                { "Type", new("EDGE") },
                                { "Name", new(dict.Name) }
                            }
                        }
                    }
                }
            };

            _graphClient.Query(Arg.Any<string>(), Arg.Is<string>(t => t.Contains(dict.Id.ToString())))
                .Returns(resultSet);
        }

        foreach (var edge in unusedEdges)
        {
            var resultSet = new ResultSet();
            resultSet.Metrics = new();
            resultSet.Results = new();

            _graphClient.Query(Arg.Any<string>(), Arg.Is<string>(t => t.Contains(edge.Id.ToString())))
                .Returns(resultSet);
        }

        // act
        var result = await _sut.GetFreeEdgesIdsAsync(allEdges);
        // assert

        result.Should().BeEquivalentTo(unusedEdges);
    }

    private List<EdgeModel> GetExampleEdges()
    {
        var retVal = new List<EdgeModel>();
        for (var i = 0; i < 4; i++)
        {
            retVal.Add(new()
            {
                Id = Guid.NewGuid(),
                Name = "Edge" + (i + 1)
            });
        }

        return retVal;
    }
}