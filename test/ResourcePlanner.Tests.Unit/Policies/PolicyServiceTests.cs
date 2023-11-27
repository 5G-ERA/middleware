using FluentAssertions;
using Microsoft.Extensions.Logging;
using Microsoft.Extensions.Options;
using Middleware.CentralApi.Sdk;
using Middleware.Common.Config;
using Middleware.Common.Enums;
using Middleware.Models.Domain;
using Middleware.Models.Domain.Slice;
using Middleware.Models.Enums;
using Middleware.RedisInterface.Contracts.Mappings;
using Middleware.RedisInterface.Contracts.Responses;
using Middleware.RedisInterface.Sdk;
using Middleware.ResourcePlanner.Policies;
using NSubstitute;
using NSubstitute.ReturnsExtensions;

namespace ResourcePlanner.Tests.Unit.Policies;

//[LogTestExecution]
public class PolicyServiceTests
{
    private readonly ICentralApiClient _centralApiClientClient = Substitute.For<ICentralApiClient>();
    private readonly ILogger<PolicyBuilder> _logger = Substitute.For<ILogger<PolicyBuilder>>();

    private readonly IOptions<MiddlewareConfig> _mwOptions = Options.Create(new MiddlewareConfig
    {
        InstanceName = "testLocation",
        InstanceType = LocationType.Edge.ToString(),
        Organization = "testOrganization"
    });

    private readonly IRedisInterfaceClient _redisInterfaceClient = Substitute.For<IRedisInterfaceClient>();
    private readonly PolicyService _sut;

    public PolicyServiceTests()
    {
        IPolicyBuilder policyBuilder =
            new PolicyBuilder(_redisInterfaceClient, _mwOptions, _centralApiClientClient, _logger);
        _sut = new(policyBuilder);
    }

    [Fact]
    public async Task GetLocationAsync_ShouldReturnDefaultLocation_WhenNoLocationPoliciesAreSpecified()
    {
        //arrange
        var instances = new List<InstanceModel>
        {
            new()
            {
                Name = "Instance1",
                OnboardedTime = DateTime.Today
            }
        };
        var loc = new Location
        {
            Name = _mwOptions.Value.InstanceName,
            Organization = _mwOptions.Value.Organization
        };
        var locResp = loc.ToLocationResponse();
        _redisInterfaceClient.GetLocationByNameAsync(loc.Name).Returns(locResp);

        var expected = new PlannedLocation(loc.Id, loc.Name, LocationType.Edge);

        //act
        var result = await _sut.GetLocationAsync(instances);

        //assert
        result.Should().NotBeNull();
        result.Should().BeEquivalentTo(expected);
    }

    [Fact]
    public async Task GetLocationAsync_ShouldReturnUrllcLocation_WhenUrllcLocationPolicyIsGiven()
    {
        //arrange
        var instances = new List<InstanceModel>
        {
            new()
            {
                Name = "Instance1",
                OnboardedTime = DateTime.Today,
                AppliedPolicies = new()
                {
                    "UrllcSliceLocation"
                }
            }
        };
        var slice = new SliceModel
        {
            Name = "UrllcSlice",
            Jitter = 10,
            Latency = 10,
            TrafficType = TrafficType.Tcp
        };
        var edge = new EdgeModel
        {
            Name = _mwOptions.Value.InstanceName,
            Organization = _mwOptions.Value.Organization
        };

        var policy = CreateLocationPolicy("UrllcSliceLocation", Priority.Normal);

        var relations = new List<RelationModel>
        {
            new()
            {
                InitiatesFrom = new(edge.Id, edge.Name, edge.GetType()),
                PointsTo = new(slice.Id, slice.Name, slice.GetType()),
                RelationName = "OFFERS"
            }
        };
        var edgeResp = edge.ToEdgeResponse();
        var policyResp = policy.ToPolicyResponse();
        var slices = new GetSlicesResponse
        {
            Slices = new List<SliceResponse> { slice.ToSliceResponse() }
        };

        _redisInterfaceClient.SliceGetAllAsync().Returns(slices);
        _redisInterfaceClient.GetEdgeByNameAsync(edge.Name).Returns(edgeResp);
        _redisInterfaceClient.GetPolicyByNameAsync(policy.Name).Returns(policyResp);
        _redisInterfaceClient.GetRelationAsync(Arg.Is<SliceModel>(t => t.Id == slice.Id), "OFFERS",
                RelationDirection.Incoming.ToString())
            .Returns(relations);

        var centralApiResponse = TestObjectBuilder.ExampleLocationsResponse(edge);
        _centralApiClientClient.GetAvailableLocations().Returns(centralApiResponse);
        var expected = new PlannedLocation(edge.Id, edge.Name, LocationType.Edge, slice.Name);
        //act

        var result = await _sut.GetLocationAsync(instances);

        //assert
        result.Should().NotBeNull();
        result.Should().BeEquivalentTo(expected);
    }

    [Fact]
    public async Task GetLocationAsync_ShouldReturnUrllcLocation_WhenUrllcAndDefaultPolicyIsGiven()
    {
        //arrange
        var instances = new List<InstanceModel>
        {
            new()
            {
                Name = "Instance1",
                OnboardedTime = DateTime.Today,
                AppliedPolicies = new()
                {
                    "UrllcSliceLocation", "DefaultLocation"
                }
            }
        };
        var slice = new SliceModel
        {
            Name = "UrllcSlice",
            Jitter = 10,
            Latency = 10,
            TrafficType = TrafficType.Tcp
        };
        var edge = new Location
        {
            Name = _mwOptions.Value.InstanceName,
            Organization = _mwOptions.Value.Organization,
            Type = LocationType.Edge
        };
        var edge2 = new Location
        {
            Name = "otherEdgeWithSlice",
            Organization = _mwOptions.Value.Organization,
            Type = LocationType.Edge
        };
        var policy = CreateLocationPolicy("UrllcSliceLocation", Priority.High);
        var policy2 = CreateLocationPolicy("DefaultLocation", Priority.Normal);

        var relations = new List<RelationModel>
        {
            new()
            {
                InitiatesFrom = new(edge2.Id, edge2.Name, edge2.GetType()),
                PointsTo = new(slice.Id, slice.Name, slice.GetType()),
                RelationName = "OFFERS"
            }
        };
        var edgeResp = edge.ToLocationResponse();
        var edge2Resp = edge2.ToLocationResponse();
        var policyResp = policy.ToPolicyResponse();
        var defaultPolicyResp = policy2.ToPolicyResponse();
        var slices = new GetSlicesResponse
        {
            Slices = new List<SliceResponse> { slice.ToSliceResponse() }
        };

        _redisInterfaceClient.SliceGetAllAsync().Returns(slices);
        _redisInterfaceClient.GetLocationByNameAsync(edge.Name).Returns(edgeResp);
        _redisInterfaceClient.GetLocationByNameAsync(edge2.Name).Returns(edge2Resp);
        _redisInterfaceClient.GetPolicyByNameAsync(policy.Name).Returns(policyResp);
        _redisInterfaceClient.GetPolicyByNameAsync(policy2.Name).Returns(defaultPolicyResp);
        _redisInterfaceClient.GetRelationAsync(Arg.Is<SliceModel>(t => t.Id == slice.Id), "OFFERS",
                RelationDirection.Incoming.ToString())
            .Returns(relations);

        var centralApiResponse = TestObjectBuilder.ExampleLocationsResponse(edge, edge2);
        _centralApiClientClient.GetAvailableLocations().Returns(centralApiResponse);

        var expected = new PlannedLocation(edge2.Id, edge2.Name, LocationType.Edge, slice.Name);

        //act
        var result = await _sut.GetLocationAsync(instances);

        //assert
        result.Should().NotBeNull();
        result.Should().BeEquivalentTo(expected);
    }

    [Fact]
    public async Task GetLocationAsync_ShouldReturnDefaultLocation_WhenIncorrectPolicyNamesAreGiven()
    {
        //arrange
        var instances = new List<InstanceModel>
        {
            new()
            {
                Name = "Instance1",
                OnboardedTime = DateTime.Today,
                AppliedPolicies = new()
                {
                    "NotExistingPolicy"
                }
            }
        };

        var defaultLoc = new Location
        {
            Name = _mwOptions.Value.InstanceName,
            Organization = _mwOptions.Value.Organization
        };
        var defaultEdgeResp = defaultLoc.ToLocationResponse();
        _redisInterfaceClient.GetLocationByNameAsync(defaultLoc.Name).Returns(defaultEdgeResp);
        _redisInterfaceClient.GetPolicyByNameAsync("NotExistingPolicy").ReturnsNull();
        var expected = new PlannedLocation(defaultLoc.Id, defaultLoc.Name, LocationType.Edge);

        //act
        var result = await _sut.GetLocationAsync(instances);

        //assert
        result.Should().NotBeNull();
        result.Should().BeEquivalentTo(expected);
    }

    [Fact]
    public async Task GetLocationAsync_ShouldReturnUrllcLocation_WhenPoliciesHaveSamePriorities()
    {
        //arrange
        var instances = new List<InstanceModel>
        {
            new()
            {
                Name = "Instance1",
                OnboardedTime = DateTime.Today,
                AppliedPolicies = new()
                {
                    "DefaultLocation", "UrllcSliceLocation"
                }
            }
        };
        var slice = new SliceModel
        {
            Name = "UrllcSlice",
            Jitter = 10,
            Latency = 10,
            TrafficType = TrafficType.Tcp
        };
        var defaultEdge = new Location
        {
            Name = _mwOptions.Value.InstanceName,
            Organization = _mwOptions.Value.Organization,
            Type = LocationType.Edge
        };
        var sliceEdge = new Location
        {
            Name = "otherEdgeWithSlice",
            Organization = _mwOptions.Value.Organization,
            Type = LocationType.Edge
        };
        var slicePolicy = CreateLocationPolicy("UrllcSliceLocation", Priority.Low);
        var defaultPolicy = CreateLocationPolicy("DefaultLocation", Priority.Low);

        var relations = new List<RelationModel>
        {
            new()
            {
                InitiatesFrom = new(sliceEdge.Id, sliceEdge.Name, sliceEdge.GetType()),
                PointsTo = new(slice.Id, slice.Name, slice.GetType()),
                RelationName = "OFFERS"
            }
        };
        var defaultEdgeResp = defaultEdge.ToLocationResponse();
        var sliceEdgeResp = sliceEdge.ToLocationResponse();
        var slicePolicyResp = slicePolicy.ToPolicyResponse();
        var defaultPolicyResp = defaultPolicy.ToPolicyResponse();
        var sliceResponse = slice.ToSliceResponse();
        var slices = new GetSlicesResponse
        {
            Slices = new List<SliceResponse> { sliceResponse }
        };

        _redisInterfaceClient.SliceGetAllAsync().Returns(slices);
        _redisInterfaceClient.GetLocationByNameAsync(defaultEdge.Name).Returns(defaultEdgeResp);
        _redisInterfaceClient.GetLocationByNameAsync(sliceEdge.Name).Returns(sliceEdgeResp);
        _redisInterfaceClient.GetPolicyByNameAsync(slicePolicy.Name).Returns(slicePolicyResp);
        _redisInterfaceClient.GetPolicyByNameAsync(defaultPolicy.Name).Returns(defaultPolicyResp);
        _redisInterfaceClient.GetRelationAsync(Arg.Is<SliceModel>(t => t.Id == slice.Id), "OFFERS",
                RelationDirection.Incoming.ToString())
            .Returns(relations);
        _redisInterfaceClient.GetRelationAsync(Arg.Is<Location>(t => t.Id == sliceEdge.Id), "OFFERS")
            .Returns(relations);
        _redisInterfaceClient.SliceGetByIdAsync(slice.Id).Returns(sliceResponse);
        var expected = new PlannedLocation(sliceEdge.Id, sliceEdge.Name, LocationType.Edge, slice.Name);

        var centralApiResponse = TestObjectBuilder.ExampleLocationsResponse(defaultEdge, sliceEdge);
        _centralApiClientClient.GetAvailableLocations().Returns(centralApiResponse);

        //act
        var result = await _sut.GetLocationAsync(instances);

        //assert
        result.Should().NotBeNull();
        result.Should().BeEquivalentTo(expected,
            "SliceLocation matches more policies than the default location. The default location is not compatible with the policy that requires Slicing mechanism implemented.");
    }

    private static PolicyModel CreateLocationPolicy(string name, Priority priority)
    {
        return new()
        {
            IsActive = true,
            Name = name,
            Priority = priority,
            Scope = PolicyScope.Resource,
            Type = PolicyType.LocationSelection
        };
    }
}