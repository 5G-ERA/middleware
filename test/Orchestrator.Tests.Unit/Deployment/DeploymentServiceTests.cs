using System;
using System.Linq;
using System.Threading.Tasks;
using k8s;
using Microsoft.Extensions.Logging;
using Microsoft.Extensions.Options;
using Middleware.Common.Config;
using Middleware.Models.Domain;
using Middleware.Orchestrator.Deployment;
using Middleware.Orchestrator.Publishers;
using Middleware.RedisInterface.Sdk;
using NSubstitute;
using Xunit;

namespace Orchestrator.Tests.Unit.Deployment;

public class DeploymentServiceTests
{
    private readonly IKubernetes _kube = Substitute.For<IKubernetes>();
    private readonly IKubernetesBuilder _kubeBuilder = Substitute.For<IKubernetesBuilder>();
    private readonly IKubernetesObjectBuilder _kubernetesObjectBuilder = Substitute.For<IKubernetesObjectBuilder>();
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

    private readonly DeploymentService _sut;

    public DeploymentServiceTests()
    {
        _kubeBuilder.CreateKubernetesClient().Returns(_kube);
        _sut = new(_kubeBuilder, _logger, _redisInterface, _mwConfig, _kubernetesObjectBuilder,
            _rosConnection, _publishingService);
    }

    [Fact]
    public async Task DeletePlanAsync_ShouldAlwaysDeleteActionPlan_WhenCalled()
    {
        //arrange
        var actionPlan = CreateActionPlan();
        var instanceId = actionPlan.ActionSequence!.First().Services.First().ServiceInstanceId;
        //_kube.AppsV1
        //    .ListNamespacedDeploymentAsync("middleware",
        //        labelSelector: KubernetesObjectExtensions.GetNetAppLabelSelector(instanceId))
        //    .Returns(new V1DeploymentList());
        //act

        //assert
        await _redisInterface.Received(1).ActionPlanDeleteAsync(actionPlan.Id);
    }

    private ActionPlanModel CreateActionPlan()
    {
        return new()
        {
            Id = Guid.NewGuid(),
            Name = "TST",
            RobotId = Guid.NewGuid(),
            TaskId = Guid.NewGuid(),
            TaskStartedAt = DateTime.Now,
            ActionSequence = new()
            {
                new()
                {
                    Name = "Action1",
                    Id = Guid.NewGuid(),
                    Placement = "test-MW",
                    PlacementType = "Edge",
                    SingleNetAppEntryPoint = true,
                    Services = new()
                    {
                        new()
                        {
                            Id = Guid.NewGuid(),
                            Name = "Instance1",
                            RosDistro = RosDistro.Foxy.ToString(),
                            IsReusable = true,
                            ServiceInstanceId = Guid.NewGuid()
                        }
                    }
                }
            }
        };
    }
}