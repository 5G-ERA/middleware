using System.Collections.Immutable;
using FluentAssertions;
using Middleware.CentralApi.Domain;
using Middleware.CentralApi.Services;
using Middleware.DataAccess.Repositories.Abstract;
using Middleware.Models.Domain;
using Middleware.Models.Enums;
using NSubstitute;
using OneOf.Types;

namespace CentralApi.Tests.Unit;

public class LocationServiceTests
{
    private readonly LocationService _sut;
    private readonly ICloudRepository _cloudRepository = Substitute.For<ICloudRepository>();
    private readonly IEdgeRepository _edgeRepository = Substitute.For<IEdgeRepository>();
    private readonly Random _rng = new Random(420);
    
    public LocationServiceTests()
    {
        _sut = new LocationService(_cloudRepository, _edgeRepository);
    }

    [Fact]
    public async Task RegisterLocation_ShouldMakeEdgeOnline_WhenExistingLocationIsEdge()
    {
        // arrange
        var paramLocation = new Location()
        {
            Name = "TestEdge",
            Organization = "MiddlewareTesting",
            Type = LocationType.Edge
        };
        var id = Guid.NewGuid();
        var edge = new EdgeModel()
        {
            Id = id,
            Name = "TestEdge",
            Organization = "MiddlewareTesting",
            EdgeIp = new Uri("https://xkcd.com/927/")
        };
        var expectedLocation = new Location()
        {
            Id = id,
            Name = "TestEdge",
            Organization = "MiddlewareTesting",
            Address = new Uri("https://xkcd.com/927/"),
            Type = LocationType.Edge
        };
        _cloudRepository.CheckIfNameExists(Arg.Any<string>()).Returns((false, null));
        _edgeRepository.CheckIfNameExists(Arg.Any<string>()).Returns((true, edge));
        _edgeRepository.AddAsync(Arg.Any<EdgeModel>()).ReturnsForAnyArgs(edge);
        // act
        var result = await _sut.RegisterLocation(paramLocation);
        // assert

        result.IsT0.Should().BeTrue();
        result.IsT1.Should().BeFalse();
        result.IsT2.Should().BeFalse();

        var resultType = result.AsT0;
        resultType.Should().BeEquivalentTo(expectedLocation);

        await _edgeRepository.Received(1).AddAsync(Arg.Any<EdgeModel>());
        await _cloudRepository.Received(0).AddAsync(Arg.Any<CloudModel>());
    }

    [Fact]
    public async Task RegisterLocation_ShouldMakeCloudOnline_WhenExistingLocationIsCloud()
    {
        // arrange
        var paramLocation = new Location()
        {
            Name = "TestCloud",
            Organization = "MiddlewareTesting",
            Type = LocationType.Cloud
        };
        var id = Guid.NewGuid();
        var cloud = new CloudModel()
        {
            Id = id,
            Name = "TestCloud",
            Organization = "MiddlewareTesting",
            CloudIp = new Uri("https://xkcd.com/927/")
        };
        var expectedLocation = new Location()
        {
            Id = id,
            Name = "TestCloud",
            Organization = "MiddlewareTesting",
            Address = new Uri("https://xkcd.com/927/"),
            Type = LocationType.Cloud
        };
        _cloudRepository.CheckIfNameExists(Arg.Any<string>()).Returns((true, cloud));
        _edgeRepository.CheckIfNameExists(Arg.Any<string>()).Returns((false, null));
        _cloudRepository.AddAsync(Arg.Any<CloudModel>()).ReturnsForAnyArgs(cloud);
        // act
        var result = await _sut.RegisterLocation(paramLocation);
        // assert

        result.IsT0.Should().BeTrue();
        result.IsT1.Should().BeFalse();
        result.IsT2.Should().BeFalse();

        var resultType = result.AsT0;
        resultType.Should().BeEquivalentTo(expectedLocation);

        await _cloudRepository.Received(1).AddAsync(Arg.Any<CloudModel>());
        await _edgeRepository.Received(0).AddAsync(Arg.Any<EdgeModel>());
    }

    [Fact]
    public async Task RegisterLocation_ShouldRegisterNewBareLocation_WhenLocationIsNotFound()
    {
        // arrange
        var paramLocation = new Location()
        {
            Name = "NotExistingLocation",
            Organization = "MiddlewareTesting",
            Type = LocationType.Cloud
        };
        _cloudRepository.CheckIfNameExists(Arg.Any<string>()).Returns((false, null));
        _edgeRepository.CheckIfNameExists(Arg.Any<string>()).Returns((false, null));
        // act
        var result = await _sut.RegisterLocation(paramLocation);
        // assert
        result.IsT0.Should().BeTrue();
        result.IsT1.Should().BeFalse();
        result.IsT2.Should().BeFalse();

        var resultType = result.AsT0;
        resultType.Should().BeOfType<Location>();

        await _cloudRepository.Received(1).AddAsync(Arg.Any<CloudModel>());
        await _edgeRepository.Received(0).AddAsync(Arg.Any<EdgeModel>());
    }

    [Fact]
    public async Task GetAvailableLocations_ShouldGetAvailableLocations_WhenGivenOrganizationName()
    {
        
        // arrange
        var org = "testOrganization";
        var edge = new EdgeModel()
        {
            Id = Guid.NewGuid(),
            Name = "TestEdge",
            Organization = org,
        };
        var cloud = new CloudModel()
        {
            Id = Guid.NewGuid(),
            Name = "TestCloud",
            Organization = org
        };
        var locations = new List<Location>()
        {
            new Location()
            {
                Id = cloud.Id,
                Name = cloud.Name,
                Organization = cloud.Organization,
                Type = LocationType.Cloud
            },
            new Location()
            {
                Id = edge.Id,
                Name = edge.Name,
                Organization = edge.Organization,
                Type = LocationType.Edge
            }
        }.ToImmutableList();
        
        _cloudRepository.GetCloudsByOrganizationAsync(org).Returns(new List<CloudModel>(){cloud}.ToImmutableList());
        _edgeRepository.GetEdgesByOrganizationAsync(org).Returns(new List<EdgeModel>(){edge}.ToImmutableList());
        
        // act
        var result = await _sut.GetAvailableLocations(org);
        
        // assert
        result.IsT0.Should().BeTrue();
        result.IsT1.Should().BeFalse();

        var resultType = result.AsT0;
        resultType.Should().BeOfType<ImmutableList<Location>>();
        
        resultType.Should().BeEquivalentTo(locations);
    }
    [Fact]
    public async Task GetAvailableLocations_ShouldReturnNotFouc_WhenOrganizationHasNoLocations()
    {
        // arrange
        var org = "testOrganization";
        var locations = ImmutableList<Location>.Empty;
        _cloudRepository.GetCloudsByOrganizationAsync(org).Returns(ImmutableList<CloudModel>.Empty);
        _edgeRepository.GetEdgesByOrganizationAsync(org).Returns(ImmutableList<EdgeModel>.Empty);
        
        // act
        var result = await _sut.GetAvailableLocations(org);
        
        // assert
        result.IsT0.Should().BeFalse();
        result.IsT1.Should().BeTrue();

        var resultType = result.AsT1;
        resultType.Should().BeOfType<NotFound>();
    }
}

