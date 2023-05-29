using FluentAssertions;
using Microsoft.Extensions.Options;
using Middleware.Common.Config;
using Middleware.Models.Domain;
using Middleware.Models.Enums;
using Middleware.RedisInterface.Contracts.Responses;
using Middleware.RedisInterface.Sdk;
using Middleware.ResourcePlanner.Policies.LocationSelection;
using NSubstitute;

namespace ResourcePlanner.Tests.Unit.Policies;

public class DefaultLocationPolicyTests
{
    private readonly IOptions<MiddlewareConfig> _mwOptions = Substitute.For<IOptions<MiddlewareConfig>>();
    private readonly IRedisInterfaceClient _redisInterfaceClient = Substitute.For<IRedisInterfaceClient>();
    private readonly DefaultLocation _sut;

    public DefaultLocationPolicyTests()
    {
        _sut = new(_mwOptions, _redisInterfaceClient);
    }

    [Fact]
    public void Priority_ShouldAlwaysBeLow()
    {
        var result = _sut.Priority;

        result.Should().Be(Priority.Low);
    }

    [Fact]
    public void FoundMatchingLocation_ShouldBeFalse_WhenGetLocationAsyncWasNotCalled()
    {
        var result = _sut.FoundMatchingLocation;

        result.Should().Be(false);
    }

    [Fact]
    public async Task FoundMatchingLocation_ShouldBeTrue_WhenGetLocationAsyncReturnedLocation()
    {
        var config = new MiddlewareConfig
        {
            InstanceName = "testInstance",
            Organization = "testOrg",
            InstanceType = LocationType.Cloud.ToString()
        };
        var cloud = new CloudResponse
        {
            Id = Guid.NewGuid(),
            Name = config.InstanceName,
            Organization = config.Organization,
            LastUpdatedTime = DateTime.Now
        };
        _mwOptions.Value.Returns(config);
        _redisInterfaceClient.GetCloudByNameAsync(config.InstanceName).Returns(cloud);

        //act
        await _sut.GetLocationAsync();
        var result = _sut.FoundMatchingLocation;
        //assert
        result.Should().Be(true);
    }

    [Fact]
    public async Task FoundMatchingLocation_ShouldReturnCurrentLocation_WhenLocationIsEdge()
    {
        var config = new MiddlewareConfig
        {
            InstanceName = "testInstance",
            Organization = "testOrg",
            InstanceType = LocationType.Edge.ToString()
        };
        var edge = new EdgeResponse
        {
            Id = Guid.NewGuid(),
            Name = config.InstanceName,
            Organization = config.Organization,
            LastUpdatedTime = DateTime.Now
        };
        _mwOptions.Value.Returns(config);
        _redisInterfaceClient.GetEdgeByNameAsync(config.InstanceName).Returns(edge);

        //act
        var result = await _sut.GetLocationAsync();

        //assert
        result.Should().BeOfType<PlannedLocation>();

        result.Id.Should().Be(edge.Id);
        result.Name.Should().Be(config.InstanceName);
        result.Type.Should().Be(LocationType.Edge);
    }

    [Fact]
    public async Task FoundMatchingLocation_ShouldAlwaysReturnLocationWithoutSlices()
    {
        var config = new MiddlewareConfig
        {
            InstanceName = "testInstance",
            Organization = "testOrg",
            InstanceType = LocationType.Edge.ToString()
        };
        var edge = new EdgeResponse
        {
            Id = Guid.NewGuid(),
            Name = config.InstanceName,
            Organization = config.Organization,
            LastUpdatedTime = DateTime.Now
        };
        _mwOptions.Value.Returns(config);
        _redisInterfaceClient.GetEdgeByNameAsync(config.InstanceName).Returns(edge);

        //act
        var result = await _sut.GetLocationAsync();

        //assert
        result.Should().BeOfType<PlannedLocation>();

        result.HasSlicesEnabled.Should().Be(false);
        result.NetworkSliceName.Should().BeNullOrEmpty();
    }


    [Fact]
    public async Task IsLocationSatisfiedByPolicy_ShouldAlwaysAcceptGivenLocation()
    {
        var result = await _sut.IsLocationSatisfiedByPolicy(Arg.Any<PlannedLocation>());

        result.Should().BeTrue();
    }
}