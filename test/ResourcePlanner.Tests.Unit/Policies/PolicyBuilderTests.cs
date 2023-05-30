using FluentAssertions;
using Microsoft.Extensions.Options;
using Middleware.Common.Config;
using Middleware.Models.Enums;
using Middleware.RedisInterface.Contracts.Responses;
using Middleware.RedisInterface.Sdk;
using Middleware.ResourcePlanner.Policies;
using Middleware.ResourcePlanner.Policies.LocationSelection;
using NSubstitute;
using NSubstitute.ReturnsExtensions;

namespace ResourcePlanner.Tests.Unit.Policies;

public class PolicyBuilderTests
{
    private readonly IOptions<MiddlewareConfig> _mwOptions = Substitute.For<IOptions<MiddlewareConfig>>();
    private readonly IRedisInterfaceClient _redisInterfaceClient = Substitute.For<IRedisInterfaceClient>();
    private readonly PolicyBuilder _sut;

    public PolicyBuilderTests()
    {
        _sut = new(_redisInterfaceClient, _mwOptions);
    }

    [Fact]
    public async Task CreateLocationPolicy_ShouldReturnNull_WhenSpecifiedPolicyIsNotLocationPolicy()
    {
        var policy = new PolicyResponse
        {
            Id = Guid.NewGuid(),
            IsActive = true,
            Name = "DefinitelyNotLocationPolicy",
            Type = PolicyType.None.ToString(),
            Priority = Priority.Normal.ToString(),
            Scope = PolicyScope.Resource.ToString()
        };
        _redisInterfaceClient.GetPolicyByNameAsync(policy.Name).Returns(policy);

        //act
        var result = await _sut.CreateLocationPolicy(policy.Name);

        result.Should().BeNull();
    }

    [Fact]
    public async Task CreateLocationPolicy_ShouldReturnNull_WhenSpecifiedPolicyIsInactive()
    {
        var policy = new PolicyResponse
        {
            Id = Guid.NewGuid(),
            IsActive = false,
            Name = "SomePolicy",
            Type = PolicyType.LocationSelection.ToString(),
            Priority = Priority.Normal.ToString(),
            Scope = PolicyScope.Resource.ToString()
        };
        _redisInterfaceClient.GetPolicyByNameAsync(policy.Name).Returns(policy);

        //act
        var result = await _sut.CreateLocationPolicy(policy.Name);

        result.Should().BeNull();
    }

    [Fact]
    public async Task CreateLocationPolicy_ShouldReturnNull_WhenSpecifiedPolicyDoesntExist()
    {
        var policy = new PolicyResponse
        {
            Id = Guid.NewGuid(),
            IsActive = false,
            Name = "SomePolicy",
            Type = PolicyType.LocationSelection.ToString(),
            Priority = Priority.Normal.ToString(),
            Scope = PolicyScope.Resource.ToString()
        };
        _redisInterfaceClient.GetPolicyByNameAsync(Arg.Any<string>())
            .ReturnsNull();

        //act
        var result = await _sut.CreateLocationPolicy(policy.Name);

        result.Should().BeNull();
    }

    [Fact]
    public async Task CreateLocationPolicy_ShouldThrowException_WhenPolicyNameIsNotPassed()
    {
        //act
        var result = () => _sut.CreateLocationPolicy(null);

        await result.Should().ThrowAsync<ArgumentException>()
            .WithMessage("Value cannot be null or whitespace. (Parameter 'policyName')");
    }

    [Fact]
    public async Task CreateLocationPolicy_ShouldReturnUrllcSliceLocation_WhenUrllcSliceLocationPolicyNameIsGiven()
    {
        var policy = new PolicyResponse
        {
            Id = Guid.NewGuid(),
            IsActive = true,
            Name = "UrllcSliceLocation",
            Type = PolicyType.LocationSelection.ToString(),
            Priority = Priority.High.ToString(),
            Scope = PolicyScope.Resource.ToString()
        };
        _redisInterfaceClient.GetPolicyByNameAsync(policy.Name).Returns(policy);

        //act
        var result = await _sut.CreateLocationPolicy(policy.Name);

        result.Should().NotBeNull();
        result.Should().BeOfType<UrllcSliceLocation>();
        result.Priority.Should().Be(Priority.High);
    }

    [Fact]
    public async Task CreateLocationPolicy_ShouldReturnPolicyFromCache_WhenPolicyHasAlreadyBeenRequested()
    {
        var policy = new PolicyResponse
        {
            Id = Guid.NewGuid(),
            IsActive = true,
            Name = "UrllcSliceLocation",
            Type = PolicyType.LocationSelection.ToString(),
            Priority = Priority.High.ToString(),
            Scope = PolicyScope.Resource.ToString()
        };
        _redisInterfaceClient.GetPolicyByNameAsync(policy.Name).Returns(policy);
        var firstRequestedPolicy = await _sut.CreateLocationPolicy(policy.Name);
        //act
        var result = await _sut.CreateLocationPolicy(policy.Name);

        result.Should().NotBeNull();
        result.Should().BeOfType<UrllcSliceLocation>();
        result.Should().BeSameAs(firstRequestedPolicy);
    }


    [Fact]
    public void GetDefaultLocationPolicy_ShouldReturnDefaultLocationPolicyImplementation()
    {
        //act
        var result = _sut.GetDefaultLocationPolicy();

        //assert
        result.Should().NotBeNull();
        result.Should().BeOfType<DefaultLocation>();
    }

    [Theory]
    [InlineData(Priority.Critical, Priority.Critical)]
    [InlineData(Priority.High, Priority.High)]
    [InlineData(Priority.Normal, Priority.Normal)]
    [InlineData(Priority.Low, Priority.Low)]
    public async Task CreateLocationPolicy_ShouldReturnUrllcSliceLocationWithConfiguredPriority(Priority configured,
        Priority expected)
    {
        var policy = new PolicyResponse
        {
            Id = Guid.NewGuid(),
            IsActive = true,
            Name = "UrllcSliceLocation",
            Type = PolicyType.LocationSelection.ToString(),
            Priority = configured.ToString(),
            Scope = PolicyScope.Resource.ToString()
        };
        _redisInterfaceClient.GetPolicyByNameAsync(policy.Name).Returns(policy);

        //act
        var result = await _sut.CreateLocationPolicy(policy.Name);

        result.Should().NotBeNull();
        result.Should().BeOfType<UrllcSliceLocation>();
        result.Priority.Should().Be(expected);
    }

    [Theory]
    [InlineData(Priority.Critical, Priority.Low)]
    [InlineData(Priority.High, Priority.Low)]
    [InlineData(Priority.Normal, Priority.Low)]
    [InlineData(Priority.Low, Priority.Low)]
    public async Task CreateLocationPolicy_ShouldReturnDefaultLocationWithLowPriority(Priority configured,
        Priority expected)
    {
        var policy = new PolicyResponse
        {
            Id = Guid.NewGuid(),
            IsActive = true,
            Name = "DefaultLocation",
            Type = PolicyType.LocationSelection.ToString(),
            Priority = configured.ToString(),
            Scope = PolicyScope.Resource.ToString()
        };
        _redisInterfaceClient.GetPolicyByNameAsync(policy.Name).Returns(policy);

        //act
        var result = await _sut.CreateLocationPolicy(policy.Name);

        result.Should().NotBeNull();
        result.Should().BeOfType<DefaultLocation>();
        result.Priority.Should().Be(expected);
    }
}