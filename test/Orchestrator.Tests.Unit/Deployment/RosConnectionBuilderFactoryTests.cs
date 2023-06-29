using FluentAssertions;
using Middleware.Models.Domain;
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

    [Fact]
    //[InlineData(RosDistro.Noetic, typeof(Ros1ConnectionBuilder))]
    //[InlineData(RosDistro.Foxy, typeof(Ros2ConnectionBuilder))]
    public void CreateConnectionBuilder_ShouldReturnCorrectRos1VersionBuilder()
    {
        //arrange
        var distro = RosDistro.Noetic;
        //act
        var result = _sut.CreateConnectionBuilder(distro);
        //assert

        result.Should().NotBeNull();
        result.Should().BeOfType(typeof(Ros1ConnectionBuilder));
        result.RosDistro.Should().Be(distro.Name);
        result.RosVersion.Should().Be(distro.RosVersionInt);
    }

    [Fact]
    public void CreateConnectionBuilder_ShouldReturnCorrectRos2VersionBuilder()
    {
        //arrange
        var distro = RosDistro.Foxy;
        //act
        var result = _sut.CreateConnectionBuilder(distro);
        //assert

        result.Should().NotBeNull();
        result.Should().BeOfType(typeof(Ros2ConnectionBuilder));
        result.RosDistro.Should().Be(distro.Name);
        result.RosVersion.Should().Be(distro.RosVersionInt);
    }
}