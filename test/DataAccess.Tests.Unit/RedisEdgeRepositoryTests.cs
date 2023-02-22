using FluentAssertions;
using Middleware.DataAccess.Repositories;
using Middleware.DataAccess.Repositories.Redis;
using Middleware.Models.Domain;
using NSubstitute;
using Redis.OM.Contracts;
using RedisGraphDotNet.Client;
using Serilog;
using StackExchange.Redis;

namespace DataAccess.Tests.Unit;

public class RedisEdgeRepositoryTests
{
    private readonly RedisEdgeRepository _sut;

    private readonly IRedisConnectionProvider _connectionProvider = Substitute.For<IRedisConnectionProvider>();
    private readonly IRedisGraphClient _graphClient = Substitute.For<IRedisGraphClient>();
    private readonly ILogger _logger = Substitute.For<ILogger>();

    public RedisEdgeRepositoryTests()
    {
        _sut = new RedisEdgeRepository(_connectionProvider, _graphClient, _logger);
    }

    [Fact]
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
            Properties = new Dictionary<string, RedisValue>()
            {
                { "ID", new RedisValue(guidStr) },
                { "Type", new RedisValue("INSTANCE") },
                { "Name", new RedisValue("Instance1") }
            }
        };
        
        foreach (var dict in occupiedEdges)
        {
            var resultSet = new ResultSet();
            resultSet.Metrics = new QueryExecutionMetrics();
            resultSet.Results = new Dictionary<string, List<RedisGraphResult>>()
            {
                {
                    "initiatesFrom", new List<RedisGraphResult>(new[] { dummyNode })
                },
                {
                    "PointsTo", new List<RedisGraphResult>
                    {
                        new Node(rng.Next())
                        {
                            Properties = new Dictionary<string, RedisValue>()
                            {
                                { "ID", new RedisValue(dict.Id.ToString()) },
                                { "Type", new RedisValue("EDGE") },
                                { "Name", new RedisValue(dict.Name) }
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
            resultSet.Metrics = new QueryExecutionMetrics();
            resultSet.Results = new Dictionary<string, List<RedisGraphResult>>();

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
        for (int i = 0; i < 4; i++)
        {
            retVal.Add(new EdgeModel()
            {
                Id = Guid.NewGuid(),
                Name = "Edge" + (i + 1),
            });
        }

        return retVal;
    }
}