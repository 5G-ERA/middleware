using System.Collections.Generic;
using System.Linq;
using FluentAssertions;
using k8s.Models;
using Microsoft.Extensions.Configuration;
using Middleware.Common;
using Middleware.Common.Enums;
using Middleware.Common.ExtensionMethods;
using Middleware.Models.Domain;
using Middleware.Orchestrator.Deployment;
using Middleware.Orchestrator.Models;
using NSubstitute;
using Xunit;

namespace Orchestrator.Tests.Unit.Deployment;

public class KubernetesObjectBuilderTests
{
    private readonly IConfiguration _configuration = Substitute.For<IConfiguration>();
    private readonly IEnvironment _environment = Substitute.For<IEnvironment>();
    private readonly KubernetesObjectBuilder _sut;

    public KubernetesObjectBuilderTests()
    {
        _sut = new(_environment, _configuration);
    }

    [Theory]
    [InlineData("name1", K8SServiceKind.ClusterIp, "name1")]
    [InlineData("name2", K8SServiceKind.ExternalName, "name2")]
    [InlineData("name3", K8SServiceKind.LoadBalancer, "name3")]
    [InlineData("name4", K8SServiceKind.NodePort, "name4")]
    public void CreateStartupService_ShouldCreateServiceWithTheSpecifiedValues(string name, K8SServiceKind serviceType,
        string expectedName)
    {
        //arrange
        var meta = new V1ObjectMeta(name: name, labels: new Dictionary<string, string>());
        var expectedSelector = CreateDefaultSelector(name);

        //act
        var result = _sut.CreateStartupService(name, serviceType, meta);
        //assert
        result.Should().NotBeNull();
        result.Should().BeOfType(typeof(V1Service));
        result.Metadata.Name.Should().Be(expectedName);
        result.Spec.Type.Should().Be(serviceType.GetStringValue());
        result.Spec.Selector.Should().BeEquivalentTo(expectedSelector);
    }

    private static Dictionary<string, string> CreateDefaultSelector(string name)
    {
        return new() { { "app", name } };
    }

    [Fact]
    public void ConfigureCrossNetAppConnection_ShouldAddAddressesOfTheNetAppsToAllDeployments()
    {
        var instance1 = new InstanceModel
        {
            Name = "netApp1",
            ServiceInstanceId = new()
        };
        var instance2 = new InstanceModel
        {
            Name = "netApp2",
            ServiceInstanceId = new()
        };
        var deployments = new List<DeploymentPair>
        {
            CreateFromInstance(instance1),
            CreateFromInstance(instance2)
        };

        //act
        _sut.ConfigureCrossNetAppConnection(deployments);
        //assert
        deployments.Should().AllSatisfy(d => d.Deployment.Spec.Template.Spec.Containers.SelectMany(c => c.Env).Any());
    }


    private DeploymentPair CreateFromInstance(InstanceModel instance)
    {
        //TODO:
        var deployment = new V1Deployment
        {
            ApiVersion = "apps/v1",
            Spec = new()
            {
                Template = new()
                {
                    Spec = new()
                    {
                        Containers = new List<V1Container>
                        {
                            new()
                            {
                                Name = instance.Name,
                                Image = instance.Name,
                                Env = new List<V1EnvVar>()
                            }
                        }
                    }
                }
            }
        };
        var service = new V1Service
        {
            Metadata = new()
            {
                Name = instance.Name,
                Labels = CreateDefaultSelector(instance.Name!)
            },
            ApiVersion = "v1",
            Kind = "Service",
            Spec = new()
            {
                Type = K8SServiceKind.ClusterIp.GetStringValue(),
                Ports = new List<V1ServicePort> { new(80) }
            }
        };
        return new(deployment, service, instance.ServiceInstanceId, instance);
    }
}