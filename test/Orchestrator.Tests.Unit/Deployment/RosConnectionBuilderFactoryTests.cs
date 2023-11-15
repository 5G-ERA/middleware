using System.Threading.Tasks;
using FluentAssertions;
using Middleware.DataAccess.Repositories.Abstract;
using Middleware.Models.Domain;
using Middleware.Orchestrator.Deployment;
using Middleware.Orchestrator.Deployment.RosCommunication;
using NSubstitute;
using Xunit;

namespace Orchestrator.Tests.Unit.Deployment;

public class RosConnectionBuilderFactoryTests
{
    private readonly RosConnectionBuilderFactory _sut;
    private readonly ISystemConfigRepository _systemConfigRepository = Substitute.For<ISystemConfigRepository>();

    public RosConnectionBuilderFactoryTests()
    {
        _sut = new(_systemConfigRepository);
    }

    [Fact]
    //[InlineData(RosDistro.Noetic, typeof(Ros1ConnectionBuilder))]
    //[InlineData(RosDistro.Foxy, typeof(Ros2ConnectionBuilder))]
    public async Task CreateConnectionBuilder_ShouldReturnCorrectRos1VersionBuilder()
    {
        //arrange
        var distro = RosDistro.Noetic;
        var cfg = new SystemConfigModel();
        _systemConfigRepository.GetConfigAsync().Returns(cfg);
        //act
        var result = await _sut.CreateConnectionBuilder(distro);
        //assert

        result.Should().NotBeNull();
        result.Should().BeOfType(typeof(Ros1ConnectionBuilder));
        result.RosDistro.Should().Be(distro.Name);
        result.RosVersion.Should().Be(distro.RosVersionInt);
    }

    [Fact]
    public async Task CreateConnectionBuilder_ShouldReturnCorrectRos2VersionBuilder()
    {
        //arrange
        var distro = RosDistro.Foxy;
        var cfg = new SystemConfigModel();
        _systemConfigRepository.GetConfigAsync().Returns(cfg);
        //act
        var result = await _sut.CreateConnectionBuilder(distro);
        //assert

        result.Should().NotBeNull();
        result.Should().BeOfType(typeof(Ros2ConnectionBuilder));
        result.RosDistro.Should().Be(distro.Name);
        result.RosVersion.Should().Be(distro.RosVersionInt);
    }
}