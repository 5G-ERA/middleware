using FluentAssertions;
using Microsoft.Extensions.Options;
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

public class PolicyServiceTests
{
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
        IPolicyBuilder policyBuilder = new PolicyBuilder(_redisInterfaceClient, _mwOptions);
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
        var edge = new EdgeModel
        {
            Name = _mwOptions.Value.InstanceName,
            Organization = _mwOptions.Value.Organization
        };
        var edgeResp = edge.ToEdgeResponse();
        _redisInterfaceClient.GetEdgeByNameAsync(edge.Name).Returns(edgeResp);

        var expected = new PlannedLocation(edge.Id, edge.Name, LocationType.Edge);

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
        var edge = new EdgeModel
        {
            Name = _mwOptions.Value.InstanceName,
            Organization = _mwOptions.Value.Organization
        };
        var edge2 = new EdgeModel
        {
            Name = "otherEdgeWithSlice",
            Organization = _mwOptions.Value.Organization
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
        var edgeResp = edge.ToEdgeResponse();
        var edge2Resp = edge2.ToEdgeResponse();
        var policyResp = policy.ToPolicyResponse();
        var defaultPolicyResp = policy2.ToPolicyResponse();
        var slices = new GetSlicesResponse
        {
            Slices = new List<SliceResponse> { slice.ToSliceResponse() }
        };

        _redisInterfaceClient.SliceGetAllAsync().Returns(slices);
        _redisInterfaceClient.GetEdgeByNameAsync(edge.Name).Returns(edgeResp);
        _redisInterfaceClient.GetEdgeByNameAsync(edge2.Name).Returns(edge2Resp);
        _redisInterfaceClient.GetPolicyByNameAsync(policy.Name).Returns(policyResp);
        _redisInterfaceClient.GetPolicyByNameAsync(policy2.Name).Returns(defaultPolicyResp);
        _redisInterfaceClient.GetRelationAsync(Arg.Is<SliceModel>(t => t.Id == slice.Id), "OFFERS",
                RelationDirection.Incoming.ToString())
            .Returns(relations);

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

        var defaultEdge = new EdgeModel
        {
            Name = _mwOptions.Value.InstanceName,
            Organization = _mwOptions.Value.Organization
        };
        var defaultEdgeResp = defaultEdge.ToEdgeResponse();
        _redisInterfaceClient.GetEdgeByNameAsync(defaultEdge.Name).Returns(defaultEdgeResp);
        _redisInterfaceClient.GetPolicyByNameAsync("NotExistingPolicy").ReturnsNull();
        var expected = new PlannedLocation(defaultEdge.Id, defaultEdge.Name, LocationType.Edge);

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
        var defaultEdge = new EdgeModel
        {
            Name = _mwOptions.Value.InstanceName,
            Organization = _mwOptions.Value.Organization
        };
        var sliceEdge = new EdgeModel
        {
            Name = "otherEdgeWithSlice",
            Organization = _mwOptions.Value.Organization
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
        var defaultEdgeResp = defaultEdge.ToEdgeResponse();
        var sliceEdgeResp = sliceEdge.ToEdgeResponse();
        var slicePolicyResp = slicePolicy.ToPolicyResponse();
        var defaultPolicyResp = defaultPolicy.ToPolicyResponse();
        var sliceResponse = slice.ToSliceResponse();
        var slices = new GetSlicesResponse
        {
            Slices = new List<SliceResponse> { sliceResponse }
        };

        _redisInterfaceClient.SliceGetAllAsync().Returns(slices);
        _redisInterfaceClient.GetEdgeByNameAsync(defaultEdge.Name).Returns(defaultEdgeResp);
        _redisInterfaceClient.GetEdgeByNameAsync(sliceEdge.Name).Returns(sliceEdgeResp);
        _redisInterfaceClient.GetPolicyByNameAsync(slicePolicy.Name).Returns(slicePolicyResp);
        _redisInterfaceClient.GetPolicyByNameAsync(defaultPolicy.Name).Returns(defaultPolicyResp);
        _redisInterfaceClient.GetRelationAsync(Arg.Is<SliceModel>(t => t.Id == slice.Id), "OFFERS",
                RelationDirection.Incoming.ToString())
            .Returns(relations);
        _redisInterfaceClient.GetRelationAsync(Arg.Is<EdgeModel>(t => t.Id == sliceEdge.Id), "OFFERS")
            .Returns(relations);
        _redisInterfaceClient.SliceGetByIdAsync(slice.Id).Returns(sliceResponse);
        var expected = new PlannedLocation(sliceEdge.Id, sliceEdge.Name, LocationType.Edge, slice.Name);

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