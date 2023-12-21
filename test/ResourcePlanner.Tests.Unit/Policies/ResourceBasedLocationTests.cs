using FluentAssertions;
using Microsoft.Extensions.Logging;
using Middleware.CentralApi.Contracts.Responses;
using Middleware.CentralApi.Sdk;
using Middleware.Models.Domain;
using Middleware.Models.Domain.Contracts;
using Middleware.Models.Enums;
using Middleware.RedisInterface.Contracts.Responses;
using Middleware.RedisInterface.Sdk;
using Middleware.ResourcePlanner.Policies.LocationSelection;
using NSubstitute;
using LocationResponse = Middleware.CentralApi.Contracts.Responses.LocationResponse;

namespace ResourcePlanner.Tests.Unit.Policies;

public class ResourceBasedLocationTests
{
    private const Priority Priority = Middleware.Models.Enums.Priority.High;
    private readonly ICentralApiClient _centralApiClient = Substitute.For<ICentralApiClient>();

    private readonly IHardwareRequirementClaim _hwClaim = new InstanceModel
    {
        Name = "exampleInstance",
        Ram = new(2048, 4096, ResourcePriority.High),
        NumberOfCores = new(2, 4, ResourcePriority.High),
        DiskStorage = new(100, 200, ResourcePriority.Medium),
        Throughput = new(200, 400, ResourcePriority.VeryHigh),
        Latency = new(200, 100, ResourcePriority.Critical, false)
    };

    private readonly ILogger<ResourceBasedLocation> _logger = Substitute.For<ILogger<ResourceBasedLocation>>();
    private readonly IRedisInterfaceClient _redisInterfaceClient = Substitute.For<IRedisInterfaceClient>();
    private readonly ResourceBasedLocation _sut;

    public ResourceBasedLocationTests()
    {
        _sut = new(Priority, _redisInterfaceClient, _centralApiClient, _logger);
    }

    [Fact]
    public async Task GetLocationAsync_ShouldNotReturnLocationsThatAreNotInRedZone_WhenThereAreOnlyRezZoneLocations()
    {
        //arrange
        var avalLocations = new LocationsResponse
        {
            Locations = new List<LocationResponse>
            {
                new()
                {
                    Id = Guid.Parse("51EF9455-5520-436B-8E86-1A87F5C1CC17"),
                    IsOnline = true,
                    Type = LocationType.Edge.ToString(),
                    Name = "MW-UnitTest",
                    Address = new("https://127.0.0.1"),
                    Organization = "5G-ERA"
                }
            }
        };
        var allLocations = new GetLocationsResponse
        {
            Locations = new List<Middleware.RedisInterface.Contracts.Responses.LocationResponse>
            {
                new()
                {
                    Id = Guid.Parse("51EF9455-5520-436B-8E86-1A87F5C1CC17"),
                    IsOnline = true,
                    Type = LocationType.Edge.ToString(),
                    Name = "MW-UnitTest",
                    IpAddress = new("https://127.0.0.1"),
                    Organization = "5G-ERA",
                    Ram = 1024,
                    NumberOfCores = 6,
                    DiskStorage = 600,
                    Throughput = 1000,
                    Latency = 200,
                    Cpu = 1,
                    VirtualRam = 4096
                }
            }
        };
        _centralApiClient.GetAvailableLocations().Returns(avalLocations);
        _redisInterfaceClient.LocationGetAllAsync().Returns(allLocations);
        //act
        var plannedLocation = await _sut.GetLocationAsync(_hwClaim);
        //assert
        plannedLocation.Should().BeNull("No locations meet the requirements");
    }
}