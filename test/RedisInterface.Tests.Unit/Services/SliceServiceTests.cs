﻿using FluentAssertions;
using Microsoft.Extensions.Logging;
using Microsoft.Extensions.Options;
using Middleware.Common.Config;
using Middleware.DataAccess.Repositories.Abstract;
using Middleware.Models.Domain;
using Middleware.Models.Domain.Slice;
using Middleware.Models.Enums;
using Middleware.RedisInterface.Services;
using NSubstitute;

namespace RedisInterface.Tests.Unit.Services;

//[LogTestExecution]
public class SliceServiceTests
{
    private readonly ILocationRepository _locationRepository = Substitute.For<ILocationRepository>();
    private readonly ILogger<SliceService> _logger = Substitute.For<ILogger<SliceService>>();
    private readonly IOptions<MiddlewareConfig> _mwOptions = Substitute.For<IOptions<MiddlewareConfig>>();
    private readonly ISliceRepository _sliceRepository = Substitute.For<ISliceRepository>();
    private readonly SliceService _sut;

    public SliceServiceTests()
    {
        _sut = new(_sliceRepository, _locationRepository, _mwOptions, _logger);
    }

    [Fact]
    public async Task
        ReRegisterSlices_ShouldReRegisterSlicesInCurrentLocation_WhenNoLocationIsGivenAndCurrentLocationIsEdge()
    {
        //arrange
        var urllcSlice = CreateUrllcSlice("slice1");
        var embbSlice = CreateEmbbSlice("slice2");
        var slices = new List<SliceModel>
        {
            urllcSlice, embbSlice
        };
        var localMwOptions = new MiddlewareConfig
        {
            InstanceName = "TestMiddleware",
            InstanceType = "Edge",
            Organization = "TestOrg"
        };
        var edge = new Location
        {
            Id = Guid.NewGuid(),
            Name = localMwOptions.InstanceName,
            Organization = localMwOptions.Organization,
            Type = LocationType.Edge
        };
        var relationSlices = GetSlicesRelatedToLocation(edge);
        _mwOptions.Value.Returns(localMwOptions);
        _locationRepository.GetByNameAsync(edge.Name).Returns(edge);
        _locationRepository.GetRelation(edge.Id, "OFFERS").ReturnsForAnyArgs(relationSlices);

        //act
        await _sut.ReRegisterSlicesAsync(slices);

        //assert

        //deleting
        await _locationRepository.ReceivedWithAnyArgs(relationSlices.Count).DeleteRelationAsync(default!);
        await _sliceRepository.ReceivedWithAnyArgs(relationSlices.Count).DeleteByIdAsync(default!);
        //adding
        await _sliceRepository.Received().AddAsync(urllcSlice);
        await _sliceRepository.Received().AddAsync(embbSlice);
        await _locationRepository.ReceivedWithAnyArgs(2).AddRelationAsync(default!);
    }

    [Fact]
    public async Task
        ReRegisterSlices_ShouldReRegisterSlicesInCurrentLocation_WhenEdgeLocationIsGiven()
    {
        //arrange
        var urllcSlice = CreateUrllcSlice("slice1");
        var embbSlice = CreateEmbbSlice("slice2");
        var slices = new List<SliceModel>
        {
            urllcSlice, embbSlice
        };
        var localMwOptions = new MiddlewareConfig
        {
            InstanceName = "TestMiddleware",
            InstanceType = "Edge",
            Organization = "TestOrg"
        };
        var edge = new Location
        {
            Id = Guid.NewGuid(),
            Name = localMwOptions.InstanceName,
            Organization = localMwOptions.Organization,
            Type = LocationType.Edge
        };
        var location = new Location
        {
            Id = edge.Id,
            Name = edge.Name,
            Organization = edge.Organization,
            Type = LocationType.Edge
        };
        var relationSlices = GetSlicesRelatedToLocation(edge);
        _mwOptions.Value.Returns(localMwOptions);
        _locationRepository.GetByNameAsync(edge.Name).Returns(edge);
        _locationRepository.GetRelation(default!, default!).ReturnsForAnyArgs(relationSlices);

        //act
        await _sut.ReRegisterSlicesAsync(slices, location);

        //assert

        //deleting
        await _locationRepository.ReceivedWithAnyArgs(relationSlices.Count).DeleteRelationAsync(default!);
        await _sliceRepository.ReceivedWithAnyArgs(relationSlices.Count).DeleteByIdAsync(default!);
        //adding
        await _sliceRepository.Received().AddAsync(urllcSlice);
        await _sliceRepository.Received().AddAsync(embbSlice);
        await _locationRepository.ReceivedWithAnyArgs(2).AddRelationAsync(default!);
    }

    [Fact]
    public async Task
        ReRegisterSlices_ShouldReRegisterSlicesInCurrentLocation_WhenNoLocationIsGivenAndCurrentLocationIsCloud()
    {
        //arrange
        var urllcSlice = CreateUrllcSlice("slice1");
        var embbSlice = CreateEmbbSlice("slice2");
        var slices = new List<SliceModel>
        {
            urllcSlice, embbSlice
        };
        var localMwOptions = new MiddlewareConfig
        {
            InstanceName = "TestMiddleware",
            InstanceType = "Cloud",
            Organization = "TestOrg"
        };
        var cloud = new Location
        {
            Id = Guid.NewGuid(),
            Name = localMwOptions.InstanceName,
            Organization = localMwOptions.Organization
        };
        var relationSlices = GetSlicesRelatedToLocation(cloud);
        _mwOptions.Value.Returns(localMwOptions);
        _locationRepository.GetByNameAsync(cloud.Name).Returns(cloud);
        _locationRepository.GetRelation(cloud.Id, "OFFERS").Returns(relationSlices);

        //act
        await _sut.ReRegisterSlicesAsync(slices);

        //assert

        //deleting
        await _locationRepository.ReceivedWithAnyArgs(relationSlices.Count).DeleteRelationAsync(default!);
        await _sliceRepository.ReceivedWithAnyArgs(relationSlices.Count).DeleteByIdAsync(default!);
        //adding
        await _sliceRepository.Received().AddAsync(urllcSlice);
        await _sliceRepository.Received().AddAsync(embbSlice);
        await _locationRepository.ReceivedWithAnyArgs(2).AddRelationAsync(default!);
    }

    [Fact]
    public async Task
        ReRegisterSlices_ShouldReRegisterSlicesInCurrentLocation_WhenCloudLocationIsGiven()
    {
        //arrange
        var urllcSlice = CreateUrllcSlice("slice1");
        var embbSlice = CreateEmbbSlice("slice2");
        var slices = new List<SliceModel>
        {
            urllcSlice, embbSlice
        };
        var localMwOptions = new MiddlewareConfig
        {
            InstanceName = "TestMiddleware",
            InstanceType = "Cloud",
            Organization = "TestOrg"
        };
        var cloud = new Location
        {
            Id = Guid.NewGuid(),
            Name = localMwOptions.InstanceName,
            Organization = localMwOptions.Organization,
            Type = LocationType.Cloud
        };
        var location = new Location
        {
            Id = cloud.Id,
            Name = cloud.Name,
            Organization = cloud.Organization,
            Type = LocationType.Cloud
        };
        var relationSlices = GetSlicesRelatedToLocation(cloud);
        _mwOptions.Value.Returns(localMwOptions);
        _locationRepository.GetByNameAsync(cloud.Name).Returns(cloud);
        _locationRepository.GetRelation(cloud.Id, "OFFERS").ReturnsForAnyArgs(relationSlices);

        //act
        await _sut.ReRegisterSlicesAsync(slices, location);

        //assert

        //deleting
        await _locationRepository.ReceivedWithAnyArgs(relationSlices.Count).DeleteRelationAsync(default!);
        await _sliceRepository.ReceivedWithAnyArgs(relationSlices.Count).DeleteByIdAsync(default!);
        //adding
        await _sliceRepository.Received().AddAsync(urllcSlice);
        await _sliceRepository.Received().AddAsync(embbSlice);
        await _locationRepository.ReceivedWithAnyArgs(2).AddRelationAsync(default!);
    }

    [Fact]
    public async Task GetAllSlicesAsync_ShouldReturnSliceList_WhenSlicesExist()
    {
        //arrange
        var urllcSlice = CreateUrllcSlice("slice1");
        var embbSlice = CreateEmbbSlice("slice2");
        var slices = new List<SliceModel>
        {
            urllcSlice, embbSlice
        };
        _sliceRepository.GetAllAsync().Returns(slices);
        //act
        var result = await _sut.GetAllSlicesAsync();
        //assert

        result.Should().NotBeNull();
        result.Should().BeEquivalentTo(slices);
    }

    [Fact]
    public async Task GetAllSlicesAsync_ShouldReturnEmptyList_WhenNoSlicesExist()
    {
        //arrange
        var slices = new List<SliceModel>();
        _sliceRepository.GetAllAsync().Returns(slices);

        //act
        var result = await _sut.GetAllSlicesAsync();

        //assert
        result.Should().NotBeNull();
        result.Should().BeEmpty();
    }

    private static SliceModel CreateUrllcSlice(string name)
    {
        return new()
        {
            Id = Guid.NewGuid(),
            Name = name,
            ExpDataRateDl = 100,
            ExpDataRateUl = 100,
            Latency = 10,
            Jitter = 1,
            Site = "TestSite",
            TrafficType = TrafficType.Tcp,
            Imsi = new() { "imsi1", "imsi2" }
        };
    }

    private static SliceModel CreateEmbbSlice(string name)
    {
        return new()
        {
            Id = Guid.NewGuid(),
            Name = name,
            ExpDataRateDl = 100,
            ExpDataRateUl = 100,
            UserDensity = 10,
            UserSpeed = 10,
            Site = "TestSite",
            TrafficType = TrafficType.Tcp,
            Imsi = new() { "imsi3", "imsi4" }
        };
    }

    private static List<RelationModel> GetSlicesRelatedToLocation(BaseModel location)
    {
        var initiates = new GraphEntityModel(location.Id, location.Name, location.GetType());

        return new()
        {
            new(initiates, new(Guid.NewGuid(), "oldSlice1", typeof(SliceModel)), "OFFERS"),
            new(initiates, new(Guid.NewGuid(), "oldSlice2", typeof(SliceModel)), "OFFERS")
        };
    }
}