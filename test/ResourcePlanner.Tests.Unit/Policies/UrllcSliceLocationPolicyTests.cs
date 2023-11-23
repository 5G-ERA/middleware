using FluentAssertions;
using Microsoft.Extensions.Logging;
using Middleware.CentralApi.Sdk;
using Middleware.Common.Enums;
using Middleware.Models.Domain;
using Middleware.Models.Domain.Slice;
using Middleware.Models.Enums;
using Middleware.RedisInterface.Contracts.Responses;
using Middleware.RedisInterface.Sdk;
using Middleware.ResourcePlanner.Policies.LocationSelection;
using NSubstitute;
using NSubstitute.ReturnsExtensions;

namespace ResourcePlanner.Tests.Unit.Policies;

//[LogTestExecution]

public class UrllcSliceLocationPolicyTests
{
    private const Priority Priority = Middleware.Models.Enums.Priority.High;
    private readonly ICentralApiClient _centralApiClientClient = Substitute.For<ICentralApiClient>();
    private readonly ILogger _logger = Substitute.For<ILogger>();
    private readonly IRedisInterfaceClient _redisInterfaceClient = Substitute.For<IRedisInterfaceClient>();
    private readonly UrllcSliceLocation _sut;

    public UrllcSliceLocationPolicyTests()
    {
        _sut = new(Priority, _redisInterfaceClient, _centralApiClientClient, _logger);
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
        var found = _sut.FoundMatchingLocation;

        result.Should().BeNull();
        found.Should().BeFalse();
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
        var relationList = new List<RelationModel> { relation };
        _redisInterfaceClient.SliceGetAllAsync().Returns(slices);
        _redisInterfaceClient.GetRelationAsync(Arg.Is<SliceModel>(t => t.Id == lowLatencySlice.Id), "OFFERS",
                RelationDirection.Incoming.ToString())
            .Returns(relationList);

        var centralApiResponse = TestObjectBuilder.ExampleLocationsResponse(edge);
        _centralApiClientClient.GetAvailableLocations().Returns(centralApiResponse);

        // Act
        var result = await _sut.GetLocationAsync();
        var found = _sut.FoundMatchingLocation;

        // Assert
        result.Should().NotBeNull();
        result.Should().BeEquivalentTo(expectedLocation);
        result.Id.Should().Be(expectedLocation.Id);
        result.Name.Should().Be(expectedLocation.Name);
        result.Type.Should().Be(expectedLocation.Type);
        result.HasSlicesEnabled.Should().Be(true);
        result.NetworkSliceName.Should().Be(expectedLocation.NetworkSliceName);

        found.Should().BeTrue();
    }

    [Fact]
    public async Task GetLocationAsync_ShouldReturnNull_WhenSlicesReturnedNull()
    {
        // Arrange
        //GetSlicesResponse slices = null!;
        _redisInterfaceClient.SliceGetAllAsync().ReturnsNull();

        // Act
        var result = await _sut.GetLocationAsync();
        var found = _sut.FoundMatchingLocation;
        // Assert
        result.Should().BeNull();
        found.Should().BeFalse();
    }

    [Fact]
    public async Task GetLocationAsync_ShouldReturnNull_WhenNoSlicesAreConfigured()
    {
        // Arrange
        var slices = new GetSlicesResponse();
        _redisInterfaceClient.SliceGetAllAsync().Returns(slices);

        // Act
        var result = await _sut.GetLocationAsync();
        var found = _sut.FoundMatchingLocation;
        // Assert
        result.Should().BeNull();
        found.Should().BeFalse();
    }

    [Fact]
    public async Task IsLocationSatisfiedByPolicy_ShouldReturnTrue_WhenLocationHasUrllcSliceAndLocationIsCloud()
    {
        //arrange
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
            Name = "embbSlice2",
            UserDensity = 10,
            UserSpeed = 100,
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
        var cloud = new Location
        {
            Name = "testLocation"
        };
        var slices = new List<SliceResponse> { embbSlice, embbSlice2, mediumLatencySlice };
        var plannedLocation = new PlannedLocation(cloud.Id, cloud.Name, LocationType.Cloud);
        _redisInterfaceClient.SliceGetByIdAsync(embbSlice.Id).Returns(embbSlice);
        _redisInterfaceClient.SliceGetByIdAsync(embbSlice2.Id).Returns(embbSlice2);
        _redisInterfaceClient.SliceGetByIdAsync(mediumLatencySlice.Id).Returns(mediumLatencySlice);
        var relations = new List<RelationModel>();
        var relationName = "OFFERS";
        var initiates = new GraphEntityModel(cloud.Id, cloud.Name, cloud.GetType());
        foreach (var slice in slices)
        {
            var points = new GraphEntityModel(slice.Id, slice.Name, typeof(SliceModel));

            relations.Add(new(initiates, points, relationName));
        }

        var centralApiResponse = TestObjectBuilder.ExampleLocationsResponse(cloud);
        _centralApiClientClient.GetAvailableLocations().Returns(centralApiResponse);

        _redisInterfaceClient.GetRelationAsync(Arg.Any<Location>(), relationName).Returns(relations);
        //act
        var result = await _sut.IsLocationSatisfiedByPolicy(plannedLocation);
        //assert
        result.Should().BeTrue();
    }

    [Fact]
    public async Task IsLocationSatisfiedByPolicy_ShouldReturnTrue_WhenLocationHasUrllcSliceAndLocationIsEdge()
    {
        //arrange
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
            Name = "embbSlice2",
            UserDensity = 10,
            UserSpeed = 100,
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
        var edge = new Location
        {
            Name = "testLocation"
        };
        var slices = new List<SliceResponse> { embbSlice, embbSlice2, mediumLatencySlice };
        var plannedLocation = new PlannedLocation(edge.Id, edge.Name, LocationType.Edge);
        _redisInterfaceClient.SliceGetByIdAsync(embbSlice.Id).Returns(embbSlice);
        _redisInterfaceClient.SliceGetByIdAsync(embbSlice2.Id).Returns(embbSlice2);
        _redisInterfaceClient.SliceGetByIdAsync(mediumLatencySlice.Id).Returns(mediumLatencySlice);
        var relations = new List<RelationModel>();
        var relationName = "OFFERS";
        var initiates = new GraphEntityModel(edge.Id, edge.Name, edge.GetType());
        foreach (var slice in slices)
        {
            var points = new GraphEntityModel(slice.Id, slice.Name, typeof(SliceModel));

            relations.Add(new(initiates, points, relationName));
        }

        var centralApiResponse = TestObjectBuilder.ExampleLocationsResponse(edge);
        _centralApiClientClient.GetAvailableLocations().Returns(centralApiResponse);

        _redisInterfaceClient.GetRelationAsync(Arg.Any<Location>(), relationName).Returns(relations);
        //act
        var result = await _sut.IsLocationSatisfiedByPolicy(plannedLocation);
        //assert
        result.Should().BeTrue();
    }

    [Fact]
    public async Task IsLocationSatisfiedByPolicy_ShouldReturnFalse_WhenRelationsAreNull()
    {
        //arrange
        var edge = new EdgeModel
        {
            Name = "testLocation"
        };
        var plannedLocation = new PlannedLocation(edge.Id, edge.Name, LocationType.Edge);
        var relationName = "OFFERS";
        _redisInterfaceClient.GetRelationAsync(Arg.Any<EdgeModel>(), relationName)!
            .Returns(Task.FromResult<List<RelationModel>>(null!));

        var centralApiResponse = TestObjectBuilder.ExampleLocationsResponse(edge);
        _centralApiClientClient.GetAvailableLocations().Returns(centralApiResponse);

        //act
        var result = await _sut.IsLocationSatisfiedByPolicy(plannedLocation);
        //assert
        result.Should().BeFalse();
    }

    [Fact]
    public async Task IsLocationSatisfiedByPolicy_ShouldReturnFalseAndSkipNotFoundSlices_WhenSomeSlicesAreNotFound()
    {
        //arrange
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
            Name = "embbSlice2",
            UserDensity = 10,
            UserSpeed = 100,
            TrafficType = TrafficType.Tcp.ToString()
        };

        var edge = new EdgeModel
        {
            Name = "testLocation"
        };
        var slices = new List<SliceResponse> { embbSlice, embbSlice2 };
        var plannedLocation = new PlannedLocation(edge.Id, edge.Name, LocationType.Edge);
        _redisInterfaceClient.SliceGetByIdAsync(embbSlice.Id).Returns(embbSlice);
        _redisInterfaceClient.SliceGetByIdAsync(embbSlice2.Id)!.Returns(Task.FromResult<SliceResponse>(null!));
        var relations = new List<RelationModel>();
        var relationName = "OFFERS";
        var initiates = new GraphEntityModel(edge.Id, edge.Name, edge.GetType());
        foreach (var slice in slices)
        {
            var points = new GraphEntityModel(slice.Id, slice.Name, typeof(SliceModel));

            relations.Add(new(initiates, points, relationName));
        }

        var centralApiResponse = TestObjectBuilder.ExampleLocationsResponse(edge);
        _centralApiClientClient.GetAvailableLocations().Returns(centralApiResponse);

        _redisInterfaceClient.GetRelationAsync(Arg.Any<EdgeModel>(), relationName).Returns(relations);
        //act
        var result = await _sut.IsLocationSatisfiedByPolicy(plannedLocation);
        //assert
        result.Should().BeFalse();
    }

    [Fact]
    public async Task IsLocationSatisfiedByPolicy_ShouldReturnFalse_WhenNoUrllcSliceFound()
    {
        //arrange
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
            Name = "embbSlice2",
            UserDensity = 10,
            UserSpeed = 100,
            TrafficType = TrafficType.Tcp.ToString()
        };

        var edge = new EdgeModel
        {
            Name = "testLocation"
        };
        var slices = new List<SliceResponse> { embbSlice, embbSlice2 };
        var plannedLocation = new PlannedLocation(edge.Id, edge.Name, LocationType.Edge);
        _redisInterfaceClient.SliceGetByIdAsync(embbSlice.Id).Returns(embbSlice);
        _redisInterfaceClient.SliceGetByIdAsync(embbSlice2.Id).Returns(embbSlice2);
        var relations = new List<RelationModel>();
        var relationName = "OFFERS";
        var initiates = new GraphEntityModel(edge.Id, edge.Name, edge.GetType());
        foreach (var slice in slices)
        {
            var points = new GraphEntityModel(slice.Id, slice.Name, typeof(SliceModel));

            relations.Add(new(initiates, points, relationName));
        }

        var centralApiResponse = TestObjectBuilder.ExampleLocationsResponse(edge);
        _centralApiClientClient.GetAvailableLocations().Returns(centralApiResponse);
        _redisInterfaceClient.GetRelationAsync(Arg.Any<EdgeModel>(), relationName).Returns(relations);
        //act
        var result = await _sut.IsLocationSatisfiedByPolicy(plannedLocation);
        //assert
        result.Should().BeFalse();
    }
}