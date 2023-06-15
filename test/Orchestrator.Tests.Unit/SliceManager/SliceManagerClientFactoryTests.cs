using FluentAssertions;
using Microsoft.Extensions.Options;
using Middleware.Common.Config;
using Middleware.Orchestrator.SliceManager;
using Middleware.RedisInterface.Sdk;
using NSubstitute;
using Xunit;

namespace Orchestrator.Tests.Unit.SliceManager;

//[LogTestExecution]
public class SliceManagerClientFactoryTests
{
    private readonly IOptions<SliceConfig> _mwOptions = Substitute.For<IOptions<SliceConfig>>();
    private readonly IRedisInterfaceClient _redisInterfaceClient = Substitute.For<IRedisInterfaceClient>();
    private readonly SliceManagerClientFactory _sut;

    public SliceManagerClientFactoryTests()
    {
        _sut = new(_mwOptions, _redisInterfaceClient);
    }

    [Theory]
    [InlineData("testbed.asd.gr", true)]
    [InlineData("192.168.0.1", true)]
    [InlineData("", false)]
    [InlineData(null, false)]
    public void IsSlicingAvailable_ShouldValidateHostnameIsConfigured(string hostname,
        bool expected)
    {
        //arrange
        _mwOptions.Value.Returns(new SliceConfig
        {
            Hostname = hostname
        });
        //act
        var result = _sut.IsSlicingAvailable();
        //assert

        result.Should().Be(expected);
    }

    [Fact]
    public void CreateSliceManagerClient_ShouldCreateClient_WhenSlicingIsConfigured()
    {
        //arrange
        _mwOptions.Value.Returns(new SliceConfig
        {
            Hostname = "http://testbed.asd.gr"
        });
        //act
        var result = _sut.CreateSliceManagerClient();

        //assert
        result.Should().NotBeNull();
        result.Should().BeOfType(typeof(Middleware.Orchestrator.SliceManager.SliceManager));
    }

    [Fact]
    public void CreateSliceManagerClient_ShouldReturnNull_WhenSlicingIsNotConfigured()
    {
        //arrange
        _mwOptions.Value.Returns(new SliceConfig
        {
            Hostname = null
        });
        //act
        var result = _sut.CreateSliceManagerClient();

        //assert
        result.Should().BeNull();
    }
}