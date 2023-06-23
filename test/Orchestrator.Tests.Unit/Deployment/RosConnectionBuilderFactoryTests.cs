using System;
using FluentAssertions;
using Middleware.Models.Enums;
using Middleware.Orchestrator.Deployment;
using Middleware.Orchestrator.Deployment.RosCommunication;
using Xunit;

namespace Orchestrator.Tests.Unit.Deployment;

public class RosConnectionBuilderFactoryTests
{
    private readonly RosConnectionBuilderFactory _sut;

    public RosConnectionBuilderFactoryTests()
    {
        _sut = new();
    }

    [Theory]
    [InlineData(RosDistro.Noetic, typeof(Ros1ConnectionBuilder))]
    [InlineData(RosDistro.Foxy, typeof(Ros2ConnectionBuilder))]
    public void CreateConnectionBuilder_ShouldReturnCorrectRosVersionBuilder(RosDistro distro, Type expectedBuilderType)
    {
        //arrange
        //act
        var result = _sut.CreateConnectionBuilder(distro);
        //assert

        result.Should().NotBeNull();
        result.Should().BeOfType(expectedBuilderType);
        result.RosDistro.Should().Be(distro.ToString());
        result.RosVersion.Should().Be((int)distro);
    }
}