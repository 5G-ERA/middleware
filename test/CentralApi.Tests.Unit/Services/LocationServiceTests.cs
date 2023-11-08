using System.Collections.Immutable;
using FluentAssertions;
using Middleware.CentralApi.Services;
using Middleware.DataAccess.Repositories.Abstract;
using Middleware.Models.Domain;
using Middleware.Models.Enums;
using NSubstitute;
using OneOf.Types;

namespace CentralApi.Tests.Unit.Services;

//[LogTestExecution]
public class LocationServiceTests
{
    private readonly ILocationRepository _locationRepository = Substitute.For<ILocationRepository>();
    private readonly LocationService _sut;

    public LocationServiceTests()
    {
        _sut = new(_locationRepository);
    }

    [Fact]
    public async Task RegisterLocation_ShouldMakeLocationOnline_WhenLocationExists()
    {
        // arrange
        var paramLocation = new Location
        {
            Name = "TestCloud",
            Organization = "MiddlewareTesting",
            Type = LocationType.Cloud
        };
        var id = Guid.NewGuid();
        var cloud = new Location
        {
            Id = id,
            Name = "TestCloud",
            Organization = "MiddlewareTesting",
            Address = new("https://xkcd.com/927/"),
            Type = LocationType.Cloud
        };
        var expectedLocation = new Location
        {
            Id = id,
            Name = "TestCloud",
            Organization = "MiddlewareTesting",
            Address = new("https://xkcd.com/927/"),
            Type = LocationType.Cloud,
            IsOnline = true
        };
        _locationRepository.ExistsAsync(Arg.Any<string>()).Returns((true, cloud));

        _locationRepository.AddAsync(Arg.Any<Location>()).ReturnsForAnyArgs(cloud);
        // act
        var result = await _sut.RegisterLocation(paramLocation);
        // assert

        result.IsT0.Should().BeTrue();
        result.IsT1.Should().BeFalse();
        result.IsT2.Should().BeFalse();

        var resultType = result.AsT0;
        resultType.Should().BeEquivalentTo(expectedLocation);
        resultType.IsOnline.Should().BeTrue();
        await _locationRepository.ReceivedWithAnyArgs(0).AddAsync(default!);
        await _locationRepository.ReceivedWithAnyArgs(1).UpdateAsync(default!);
    }

    [Fact]
    public async Task RegisterLocation_ShouldRegisterNewBareLocation_WhenLocationIsNotFound()
    {
        // arrange
        var paramLocation = new Location
        {
            Name = "NotExistingLocation",
            Organization = "MiddlewareTesting",
            Type = LocationType.Cloud
        };
        _locationRepository.ExistsAsync(Arg.Any<string>()).Returns((false, null));

        // act
        var result = await _sut.RegisterLocation(paramLocation);
        // assert
        result.IsT0.Should().BeTrue();
        result.IsT1.Should().BeFalse();
        result.IsT2.Should().BeFalse();

        var resultType = result.AsT0;
        resultType.Should().BeOfType<Location>();

        await _locationRepository.ReceivedWithAnyArgs(1).AddAsync(default!);
    }

    [Fact]
    public async Task GetAvailableLocations_ShouldGetAvailableLocations_WhenGivenOrganizationName()
    {
        // arrange
        var org = "testOrganization";
        var edge = new Location
        {
            Id = Guid.NewGuid(),
            Name = "TestEdge",
            Organization = org,
            Type = LocationType.Edge
        };
        var cloud = new Location
        {
            Id = Guid.NewGuid(),
            Name = "TestCloud",
            Organization = org,
            Type = LocationType.Cloud
        };
        var locations = new List<Location>
        {
            new()
            {
                Id = cloud.Id,
                Name = cloud.Name,
                Organization = cloud.Organization,
                Type = LocationType.Cloud
            },
            new()
            {
                Id = edge.Id,
                Name = edge.Name,
                Organization = edge.Organization,
                Type = LocationType.Edge
            }
        }.ToImmutableList();
        var retVal = new List<Location> { cloud, edge }.ToImmutableList();

        _locationRepository.GetLocationsByOrganizationAsync(org).Returns(retVal);
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
        _locationRepository.GetLocationsByOrganizationAsync(org).Returns(ImmutableList<Location>.Empty);

        // act
        var result = await _sut.GetAvailableLocations(org);

        // assert
        result.IsT0.Should().BeFalse();
        result.IsT1.Should().BeTrue();

        var resultType = result.AsT1;
        resultType.Should().BeOfType<NotFound>();
    }
}