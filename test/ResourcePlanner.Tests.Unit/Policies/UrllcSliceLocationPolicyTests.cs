using FluentAssertions;
using Middleware.Common.Enums;
using Middleware.Models.Domain;
using Middleware.Models.Domain.Slice;
using Middleware.Models.Enums;
using Middleware.RedisInterface.Contracts.Responses;
using Middleware.RedisInterface.Sdk;
using Middleware.ResourcePlanner.Policies.LocationSelection;
using NSubstitute;

namespace ResourcePlanner.Tests.Unit.Policies;

public class UrllcSliceLocationPolicyTests
{
    private const Priority Priority = Middleware.Models.Enums.Priority.High;
    private readonly IRedisInterfaceClient _redisInterfaceClient = Substitute.For<IRedisInterfaceClient>();
    private readonly UrllcSliceLocation _sut;

    public UrllcSliceLocationPolicyTests()
    {
        _sut = new(Priority, _redisInterfaceClient);
    }

    [Fact]
    public async Task GetLocationAsync_ShouldReturnNull_WhenThereAreNoUrllcSlices()
    {
        var embbSlice = new SliceResponse
        {
            Id = Guid.NewGuid(),
            Name = "embbSlice",
            UserDensity = 10,
            UserSpeed = 100,
            TrafficType = TrafficType.Tcp.ToString()
        };
        var embbSlice2 = new SliceResponse
        {
            Id = Guid.NewGuid(),
            Name = "embbSlice",
            UserDensity = 10,
            UserSpeed = 100,
            TrafficType = TrafficType.Tcp.ToString()
        };
        var embbSlice3 = new SliceResponse
        {
            Id = Guid.NewGuid(),
            Name = "embbSlice",
            UserDensity = 10,
            UserSpeed = 100,
            TrafficType = TrafficType.Tcp.ToString()
        };
        var slices = new GetSlicesResponse
        {
            Slices = new List<SliceResponse>
            {
                embbSlice, embbSlice2, embbSlice3
            }
        };
        _redisInterfaceClient.SliceGetAllAsync().Returns(slices);

        var result = await _sut.GetLocationAsync();

        result.Should().BeNull();
    }

    [Fact]
    public async Task GetLocationAsync_ShouldReturnLocationForTheSliceWithLowestLatency()
    {
        var embbSlice = new SliceResponse
        {
            Id = Guid.NewGuid(),
            Name = "embbSlice",
            UserDensity = 10,
            UserSpeed = 100,
            TrafficType = TrafficType.Tcp.ToString()
        };
        var lowLatencySlice = new SliceResponse
        {
            Id = Guid.NewGuid(),
            Name = "lowLatencySlice",
            Latency = 5,
            Jitter = 5,
            TrafficType = TrafficType.Tcp.ToString()
        };
        var mediumLatencySlice = new SliceResponse
        {
            Id = Guid.NewGuid(),
            Name = "lowLatencySlice",
            Latency = 5,
            Jitter = 5,
            TrafficType = TrafficType.Tcp.ToString()
        };
        var edge = new EdgeModel
        {
            Name = "sliceEdge",
            Organization = "sliceOrg"
        };
        var expectedLocation = new PlannedLocation(edge.Id, edge.Name, LocationType.Edge, lowLatencySlice.Name);
        // Arrange
        var slices = new GetSlicesResponse
        {
            Slices = new List<SliceResponse>
            {
                embbSlice, lowLatencySlice, mediumLatencySlice
            }
        };
        var initiates = new GraphEntityModel(edge.Id, edge.Name, edge.GetType());
        var points = new GraphEntityModel(lowLatencySlice.Id, lowLatencySlice.Name, lowLatencySlice.GetType());
        var relation = new RelationModel(initiates, points, "OFFERS");
        _redisInterfaceClient.SliceGetAllAsync().Returns(slices);
        _redisInterfaceClient.GetRelationAsync(Arg.Any<SliceModel>(), "OFFERS", RelationDirection.Incoming.ToString())
            .Returns(new List<RelationModel> { relation });

        // Act
        var result = await _sut.GetLocationAsync();

        // Assert
        result.Should().NotBeNull();
        result.Should().BeEquivalentTo(expectedLocation);
        result.Id.Should().Be(expectedLocation.Id);
        result.Name.Should().Be(expectedLocation.Name);
        result.Type.Should().Be(expectedLocation.Type);
        result.HasSlicesEnabled.Should().Be(true);
        result.NetworkSliceName.Should().Be(expectedLocation.NetworkSliceName);
    }

    [Fact]
    public async Task GetLocationAsync_ShouldReturnNull_WhenSlicesReturnedNull()
    {
        // Arrange
        GetSlicesResponse slices = null!;
        _redisInterfaceClient.SliceGetAllAsync().Returns(slices);

        // Act
        var result = await _sut.GetLocationAsync();

        // Assert
        result.Should().BeNull();
    }

    [Fact]
    public async Task GetLocationAsync_ShouldReturnNull_WhenNoSlicesAreConfigured()
    {
        // Arrange
        var slices = new GetSlicesResponse();
        _redisInterfaceClient.SliceGetAllAsync().Returns(slices);

        // Act
        var result = await _sut.GetLocationAsync();

        // Assert
        result.Should().BeNull();
    }
}