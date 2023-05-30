using FluentAssertions;
using Microsoft.Extensions.Options;
using Middleware.Common.Config;
using Middleware.DataAccess.Repositories.Abstract;
using Middleware.Models.Domain;
using Middleware.Models.Domain.Slice;
using Middleware.Models.Enums;
using Middleware.RedisInterface.Services;
using NSubstitute;

namespace RedisInterface.Tests.Unit.Services;

public class SliceServiceTests
{
    private readonly ICloudRepository _cloudRepository = Substitute.For<ICloudRepository>();
    private readonly IEdgeRepository _edgeRepository = Substitute.For<IEdgeRepository>();

    private readonly IOptions<MiddlewareConfig> _mwOptions = Substitute.For<IOptions<MiddlewareConfig>>();

    private readonly ISliceRepository _sliceRepository = Substitute.For<ISliceRepository>();
    private readonly SliceService _sut;

    public SliceServiceTests()
    {
        _sut = new(_edgeRepository, _cloudRepository, _sliceRepository, _mwOptions);
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
        var edge = new EdgeModel
        {
            Id = Guid.NewGuid(),
            Name = localMwOptions.InstanceName,
            Organization = localMwOptions.Organization
        };
        var relationSlices = GetSlicesRelatedToLocation(edge);
        _mwOptions.Value.Returns(localMwOptions);
        _edgeRepository.GetEdgeResourceDetailsByNameAsync(Arg.Is(edge.Name)).Returns(edge);
        _edgeRepository.GetRelation(Arg.Is(edge.Id), "Offers").ReturnsForAnyArgs(relationSlices);

        //act
        await _sut.ReRegisterSlicesAsync(slices);

        //assert

        //deleting
        await _edgeRepository.Received(relationSlices.Count).DeleteRelationAsync(Arg.Any<RelationModel>());
        await _sliceRepository.Received(relationSlices.Count).DeleteByIdAsync(Arg.Any<Guid>());
        //adding
        await _sliceRepository.Received().AddAsync(urllcSlice);
        await _sliceRepository.Received().AddAsync(embbSlice);
        await _edgeRepository.Received(2).AddRelationAsync(Arg.Any<RelationModel>());
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
        var edge = new EdgeModel
        {
            Id = Guid.NewGuid(),
            Name = localMwOptions.InstanceName,
            Organization = localMwOptions.Organization
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
        _edgeRepository.GetEdgeResourceDetailsByNameAsync(Arg.Is(edge.Name)).Returns(edge);
        _edgeRepository.GetRelation(Arg.Is(edge.Id), "Offers").ReturnsForAnyArgs(relationSlices);

        //act
        await _sut.ReRegisterSlicesAsync(slices, location);

        //assert

        //deleting
        await _edgeRepository.Received(relationSlices.Count).DeleteRelationAsync(Arg.Any<RelationModel>());
        await _sliceRepository.Received(relationSlices.Count).DeleteByIdAsync(Arg.Any<Guid>());
        //adding
        await _sliceRepository.Received().AddAsync(urllcSlice);
        await _sliceRepository.Received().AddAsync(embbSlice);
        await _edgeRepository.Received(2).AddRelationAsync(Arg.Any<RelationModel>());
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
        var cloud = new CloudModel
        {
            Id = Guid.NewGuid(),
            Name = localMwOptions.InstanceName,
            Organization = localMwOptions.Organization
        };
        var relationSlices = GetSlicesRelatedToLocation(cloud);
        _mwOptions.Value.Returns(localMwOptions);
        _cloudRepository.GetCloudResourceDetailsByNameAsync(Arg.Is(cloud.Name)).Returns(cloud);
        _cloudRepository.GetRelation(Arg.Is(cloud.Id), "OFFERS").ReturnsForAnyArgs(relationSlices);

        //act
        await _sut.ReRegisterSlicesAsync(slices);

        //assert

        //deleting
        await _cloudRepository.Received(relationSlices.Count).DeleteRelationAsync(Arg.Any<RelationModel>());
        await _sliceRepository.Received(relationSlices.Count).DeleteByIdAsync(Arg.Any<Guid>());
        //adding
        await _sliceRepository.Received().AddAsync(urllcSlice);
        await _sliceRepository.Received().AddAsync(embbSlice);
        await _cloudRepository.Received(2).AddRelationAsync(Arg.Any<RelationModel>());
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
        var cloud = new CloudModel
        {
            Id = Guid.NewGuid(),
            Name = localMwOptions.InstanceName,
            Organization = localMwOptions.Organization
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
        _cloudRepository.GetCloudResourceDetailsByNameAsync(Arg.Is(cloud.Name)).Returns(cloud);
        _cloudRepository.GetRelation(Arg.Is(cloud.Id), "OFFERS").ReturnsForAnyArgs(relationSlices);

        //act
        await _sut.ReRegisterSlicesAsync(slices, location);

        //assert

        //deleting
        await _cloudRepository.Received(relationSlices.Count).DeleteRelationAsync(Arg.Any<RelationModel>());
        await _sliceRepository.Received(relationSlices.Count).DeleteByIdAsync(Arg.Any<Guid>());
        //adding
        await _sliceRepository.Received().AddAsync(urllcSlice);
        await _sliceRepository.Received().AddAsync(embbSlice);
        await _cloudRepository.Received(2).AddRelationAsync(Arg.Any<RelationModel>());
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