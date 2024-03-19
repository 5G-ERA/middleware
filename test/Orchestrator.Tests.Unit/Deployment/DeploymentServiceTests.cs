using System;
using System.Collections.Generic;
using System.IO;
using System.Linq;
using System.Text.Json;
using System.Threading.Tasks;
using FluentAssertions;
using k8s;
using Microsoft.Extensions.Logging;
using Microsoft.Extensions.Options;
using Middleware.Common;
using Middleware.Common.Config;
using Middleware.Common.Result;
using Middleware.DataAccess.Repositories.Abstract;
using Middleware.Models.Domain;
using Middleware.Orchestrator.Deployment;
using Middleware.Orchestrator.Models;
using Middleware.Orchestrator.Publishers;
using Middleware.RedisInterface.Contracts.Mappings;
using Middleware.RedisInterface.Contracts.Responses;
using Middleware.RedisInterface.Sdk;
using NSubstitute;
using Xunit;

namespace Orchestrator.Tests.Unit.Deployment;

public class DeploymentServiceTests
{
    private readonly IKubernetes _kube = Substitute.For<IKubernetes>();
    private readonly IKubernetesBuilder _kubeBuilder = Substitute.For<IKubernetesBuilder>();
    
    private readonly ILogger<DeploymentService> _logger = Substitute.For<ILogger<DeploymentService>>();

    private readonly IOptions<MiddlewareConfig> _mwConfig = Options.Create(new MiddlewareConfig
    {
        InstanceName = "test-MW",
        InstanceType = "Edge",
        Organization = "5G-ERA-TST"
    });

    private readonly IPublishingService _publishingService = Substitute.For<IPublishingService>();
    private readonly IRedisInterfaceClient _redisInterface = Substitute.For<IRedisInterfaceClient>();
    private readonly IRosConnectionBuilderFactory _rosConnection = Substitute.For<IRosConnectionBuilderFactory>();
    private readonly ISystemConfigRepository _systemConfigRepo = Substitute.For<ISystemConfigRepository>();
    private readonly IKubernetesWrapper _kubeWrapper = Substitute.For<IKubernetesWrapper>();
    private readonly IEnvironment _env = Substitute.For<IEnvironment>();
    private readonly DeploymentService _sut;

    public DeploymentServiceTests()
    {
        var kubernetesObjectBuilder = new KubernetesObjectBuilder(_env, _mwConfig);
        _kubeBuilder.CreateKubernetesClient().Returns(_kube);
        _sut = new(_logger, _redisInterface, _mwConfig, kubernetesObjectBuilder,
            _rosConnection, _publishingService, _systemConfigRepo, _kubeWrapper);
    }

    /* Also to be tested for deployment:
     *  - data persistence
     *  - inter-relay deployment
     *  - setting netapp address
     *  - name change if netapp with specified name is already deployed
     *  - error handling
     *  - ros-based NetApp deployment
     */
    /* Additional improvements:
     *  - cache read files
     *  - add IDisposable to the test class
     *
     */
    [Fact]
    public async Task DeployPlanAsync_ShouldCreateAndDeploySimpleActionPlan_WhenNetAppIsNotRosBased()
    {
        //arrange
        var task = GetTask();
        var robot = GetRobot();
        var systemConfig = GetSystemConfig();
        var location = GetLocation();
        var container = GetContainer();
        var currentlyRunningDeployments = new List<string>() { "non-existing-deployment1", "non-existing-deployment2" };
        
        _systemConfigRepo.GetConfigAsync().Returns(systemConfig);
        _kubeWrapper.GetCurrentlyDeployedNetApps().Returns(currentlyRunningDeployments);
        _redisInterface.GetLocationByNameAsync(_mwConfig.Value.InstanceName).Returns(location.ToLocationResponse());
        _redisInterface.RobotGetByIdAsync(robot.Id).Returns(robot.ToRobotResponse());
        
        var instance = task.ActionSequence!.First().Services.First();
        _redisInterface.ContainerImageGetForInstanceAsync(instance.Id).Returns(new GetContainersResponse()
            { Containers = new[] { container.ToContainerResponse() } });
        _redisInterface.AddRelationAsync(Arg.Any<InstanceModel>(), Arg.Any<Location>(), "LOCATED_AT")
            .Returns(Result.Success());
         _redisInterface.ActionPlanAddAsync(Arg.Any<ActionPlanModel>()).Returns(true);
        // act

        //act
        var result = await _sut.DeployActionPlanAsync(task, robot.Id);
        
        //assert
        result.Error.Should().BeNullOrEmpty();
        result.IsSuccess.Should().BeTrue();
        await _redisInterface.Received(1).ContainerImageGetForInstanceAsync(instance.Id);
        await _redisInterface.Received(1).AddRelationAsync(Arg.Any<InstanceModel>(), Arg.Any<Location>(), "LOCATED_AT");
        await _redisInterface.Received(1).ActionPlanAddAsync(Arg.Any<ActionPlanModel>());
        await _publishingService.Received(1)
            .PublishGatewayAddNetAppEntryAsync(Arg.Any<Location>(), Arg.Any<string>(), task.ActionPlanId, Arg.Any<Guid>());
        await _kubeWrapper.Received(1).DeployNetApp(Arg.Any<DeploymentPair>());
        currentlyRunningDeployments.Should().Contain(container.Name);
    }

    [Fact]
    public async Task DeployActionAsync_ShouldDeploySingleActionFromActionPlan_WhenRequestIsReceived()
    {
        //arrange
        var task = GetTask();
        var robot = GetRobot();
        var systemConfig = GetSystemConfig();
        var location = GetLocation();
        var container = GetContainer();
        var currentlyRunningDeployments = new List<string>() { "non-existing-deployment1", "non-existing-deployment2" };
        
        _systemConfigRepo.GetConfigAsync().Returns(systemConfig);
        _kubeWrapper.GetCurrentlyDeployedNetApps().Returns(currentlyRunningDeployments);
        _redisInterface.GetLocationByNameAsync(_mwConfig.Value.InstanceName).Returns(location.ToLocationResponse());
        _redisInterface.RobotGetByIdAsync(robot.Id).Returns(robot.ToRobotResponse());
        
        var instance = task!.ActionSequence!.First().Services.First();
        _redisInterface.ContainerImageGetForInstanceAsync(instance.Id).Returns(new GetContainersResponse()
            { Containers = new[] { container.ToContainerResponse() } });
        _redisInterface.AddRelationAsync(Arg.Any<InstanceModel>(), Arg.Any<Location>(), "LOCATED_AT")
            .Returns(Result.Success());
         _redisInterface.ActionPlanAddAsync(Arg.Any<ActionPlanModel>()).Returns(true);
        // act

        //act
        var result = await _sut.DeployActionPlanAsync(task, robot.Id);
        
        //assert
        result.Error.Should().BeNullOrEmpty();
        result.IsSuccess.Should().BeTrue();
        await _redisInterface.Received(1).ContainerImageGetForInstanceAsync(instance.Id);
        await _redisInterface.Received(1).AddRelationAsync(Arg.Any<InstanceModel>(), Arg.Any<Location>(), "LOCATED_AT");
        await _redisInterface.Received(1).ActionPlanAddAsync(Arg.Any<ActionPlanModel>());
        await _publishingService.Received(1)
            .PublishGatewayAddNetAppEntryAsync(Arg.Any<Location>(), Arg.Any<string>(), task.ActionPlanId, Arg.Any<Guid>());
        await _kubeWrapper.Received(1).DeployNetApp(Arg.Any<DeploymentPair>());
        currentlyRunningDeployments.Should().Contain(container.Name);
    }

    private static TaskModel? _task;

    private static TaskModel GetTask()
    {
        if (_task is not null)
        {
            return _task;
        }

        _task = ReadJsonFile<TaskModel>("files/task.json");
        return _task;
    }

    private static RobotModel? _robot;

    private static RobotModel GetRobot()
    {
        if (_robot is not null)
        {
            return _robot;
        }

        _robot = ReadJsonFile<RobotModel>("files/robot.json");

        return _robot;
    }

    private static SystemConfigModel? _systemConfig;

    private static SystemConfigModel GetSystemConfig()
    {
        if (_systemConfig is not null)
        {
            return _systemConfig;
        }

        _systemConfig = ReadJsonFile<SystemConfigModel>("files/config.json");

        return _systemConfig;
    }

    private static Location? _location;

    private static Location GetLocation()
    {
        if (_location is not null)
        {
            return _location;
        }

        _location = ReadJsonFile<Location>("files/location.json");

        return _location;
    }

    private static ContainerImageModel? _container;

    private static ContainerImageModel GetContainer()
    {
        if (_container is not null)
        {
            return _container;
        }

        _container = ReadJsonFile<ContainerImageModel>("files/container-image.json");

        return _container;
    }

    private static T ReadJsonFile<T>(string filePath)
    {
        var correctedPath = Path.Combine("../../../", filePath);
        string json = File.ReadAllText(correctedPath);

        return JsonSerializer.Deserialize<T>(json)!;
    }
}