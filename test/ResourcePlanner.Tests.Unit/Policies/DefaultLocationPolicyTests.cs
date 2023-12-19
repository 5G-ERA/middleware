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

//[LogTestExecution]
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
        var loc = new LocationResponse
        {
            Id = Guid.NewGuid(),
            Name = config.InstanceName,
            Organization = config.Organization,
            LastUpdatedTime = DateTime.Now,
            Type = LocationType.Cloud.ToString()
        };
        _mwOptions.Value.Returns(config);
        _redisInterfaceClient.GetLocationByNameAsync(config.InstanceName).Returns(loc);

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
        var loc = new LocationResponse
        {
            Id = Guid.NewGuid(),
            Name = config.InstanceName,
            Organization = config.Organization,
            LastUpdatedTime = DateTime.Now,
            Type = LocationType.Edge.ToString()
        };
        _mwOptions.Value.Returns(config);
        _redisInterfaceClient.GetLocationByNameAsync(config.InstanceName).Returns(loc);

        //act
        var result = await _sut.GetLocationAsync(null);

        //assert
        result.Should().BeOfType<PlannedLocation>();

        result.Id.Should().Be(loc.Id);
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
        var location = new LocationResponse
        {
            Id = Guid.NewGuid(),
            Name = config.InstanceName,
            Organization = config.Organization,
            LastUpdatedTime = DateTime.Now,
            Type = LocationType.Edge.ToString()
        };
        _mwOptions.Value.Returns(config);
        _redisInterfaceClient.GetLocationByNameAsync(config.InstanceName).Returns(location);

        //act
        var result = await _sut.GetLocationAsync(null);

        //assert
        result.Should().BeOfType<PlannedLocation>();

        result.HasSlicesEnabled.Should().Be(false);
        result.NetworkSliceName.Should().BeNullOrEmpty();
    }


    [Fact]
    public async Task IsLocationSatisfiedByPolicy_ShouldAlwaysAcceptGivenLocation()
    {
        var location = new PlannedLocation(Guid.NewGuid(), "name", LocationType.Cloud, null!, null);
        var result = await _sut.IsLocationSatisfiedByPolicy(location);

        result.Should().BeTrue();
    }
}