using System.Collections.Generic;
using System.Linq;
using FluentAssertions;
using k8s.Models;
using Middleware.Models.Enums;
using Middleware.Orchestrator.Deployment.RosCommunication;
using Xunit;

namespace Orchestrator.Tests.Unit.Deployment;

public class Ros1ConnectionBuilderTests
{
    [Fact]
    public void EnableRosCommunication_ShouldConfigureDeploymentWithNewContainer()
    {
        var distro = RosDistro.Noetic;
        //arrange
        var depl = CreateExampleDeployment();
        var sut = new Ros1ConnectionBuilder(distro);
        //act
        var result = sut.EnableRosCommunication(depl);
        //assert
        result.Spec.Template.Spec.Containers.Should()
            .HaveCount(2, "We need two containers, one RelayNetApp and container itself");
        var newContainer = result.Spec.Template.Spec.Containers.FirstOrDefault(c => c.Name == distro.ToString());

        newContainer.Should().NotBeNull();
        newContainer!.Image.Should().Be("ros:noetic-ros-core");
        newContainer.Ports.Should().HaveCount(1);
        newContainer.Ports.First().ContainerPort.Should().Be(11311, "It is default ros master port");
    }

    private V1Deployment CreateExampleDeployment()
    {
        return new()
        {
            ApiVersion = "apps/v1",
            Kind = "Deployment",
            Metadata = new()
            {
                Name = "example",
                Labels = new Dictionary<string, string>
                {
                    { "app", "example" }
                }
            },
            Spec = new()
            {
                Template = new()
                {
                    Metadata = new()
                    {
                        Name = "example",
                        Labels = new Dictionary<string, string>
                        {
                            { "app", "example" }
                        }
                    },
                    Spec = new()
                    {
                        Containers = new List<V1Container>
                        {
                            new()
                            {
                                Name = "example",
                                Image = "redis",
                                Ports = new List<V1ContainerPort>
                                {
                                    new(6379)
                                }
                            }
                        }
                    }
                }
            }
        };
    }
}