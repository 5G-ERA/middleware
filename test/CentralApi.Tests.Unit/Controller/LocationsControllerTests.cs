using System.Collections.Immutable;
using FluentAssertions;
using Microsoft.AspNetCore.Mvc;
using Middleware.CentralApi.Contracts.Responses;
using Middleware.CentralApi.Controllers;
using Middleware.CentralApi.Services;
using Middleware.Common.Responses;
using Middleware.Models.Domain;
using NSubstitute;
using OneOf.Types;

namespace CentralApi.Tests.Unit.Controller;

public class LocationsControllerTests
{
    private readonly ILocationService _locationService = Substitute.For<ILocationService>();
    private readonly LocationsController _sut;

    public LocationsControllerTests()
    {
        _sut = new(_locationService);
    }

    [Fact]
    public async Task GetAvailableLocations_ShouldReturnBadRequest_WhenLocationIsNotSpecified()
    {
        // arrange
        var location = string.Empty;
        // act
        var result = (BadRequestObjectResult)await _sut.GetAvailableLocations(location);
        // assert
        result.StatusCode.Should().Be(400);
        result.Value.Should().BeOfType<ApiResponse>();
        (result.Value as ApiResponse)!.IsError.Should().BeTrue();
        (result.Value as ApiResponse)!.StatusCode.Should().Be(400);
        (result.Value as ApiResponse)!.TimeStamp.Should().NotBe(default);
        (result.Value as ApiResponse)!.Message.Should().Be("Organization was not specified");
    }

    [Fact]
    public async Task GetAvailableLocations_ShouldReturnNotFound_WhenLocationAreNotFound()
    {
        // arrange
        var location = "myOrganization";
        _locationService.GetAvailableLocations(location).Returns(new NotFound());
        // act
        var result = (NotFoundResult)await _sut.GetAvailableLocations(location);
        // assert
        result.StatusCode.Should().Be(404);
    }

    [Fact]
    public async Task GetAvailableLocations_ShouldReturnLocations_WhenLocationsExist()
    {
        // arrange
        var location = "myOrganization";
        _locationService.GetAvailableLocations(location).Returns(ImmutableList<Location>.Empty);
        // act
        var result = (OkObjectResult)await _sut.GetAvailableLocations(location);
        // assert
        result.StatusCode.Should().Be(200);
        result.Value.Should().BeOfType<LocationsResponse>();
    }
}