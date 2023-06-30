using System;
using System.Collections.Generic;
using System.Linq;
using FluentAssertions;
using k8s.Models;
using Middleware.Models.Domain;
using Middleware.Models.Domain.ValueObjects;
using Middleware.Orchestrator.Deployment.RosCommunication;
using Xunit;

namespace Orchestrator.Tests.Unit.Deployment;

public class Ros1ConnectionBuilderTests
{
    [Fact]
    public void Ros1ConnectionBuilder_ShouldThrowArgumentExceptionWhenGivenRos2Distro()
    {
        //arrange
        var ros2Distro = RosDistro.Foxy;
        //act
        var act = () => new Ros1ConnectionBuilder(ros2Distro);
        //assert
        act.Should().Throw<ArgumentException>()
            .WithMessage(
                "Ros1ConnectionBuilder cannot provide connectivity for ROS version other than 1 (Parameter 'distro')");
    }

    [Fact]
    public void EnableRosCommunication_ShouldConfigureDeploymentWithNewContainers()
    {
        //arrange
        var distro = RosDistro.Noetic;
        var depl = CreateExampleDeployment();
        var sut = new Ros1ConnectionBuilder(distro);
        var topics = new List<RosTopicModel>();
        //act
        var result = sut.EnableRosCommunication(depl, topics);
        //assert
        result.Spec.Template.Spec.Containers.Should()
            .HaveCount(3, "We need three containers, one RelayNetApp, one ROS core and NetApp itself");
        var rosCoreContainer = result.Spec.Template.Spec.Containers.FirstOrDefault(c => c.Name == distro.ToString());

        rosCoreContainer.Should().NotBeNull();
        rosCoreContainer!.Image.Should().Be("ros:noetic-ros-core");
        rosCoreContainer.Ports.Should().HaveCount(1);
        rosCoreContainer.Ports.First().ContainerPort.Should().Be(11311, "It is default ros master port");

        var relayNetAppContainer = result.Spec.Template.Spec.Containers.FirstOrDefault(c => c.Name == "relayNetApp");

        relayNetAppContainer.Should().NotBeNull();
        relayNetAppContainer!.Image.Should().Be("but5gera/relay_network_application:0.1.0");
        relayNetAppContainer.Ports.Should().HaveCount(1);
        relayNetAppContainer.Ports.First().ContainerPort.Should().Be(80, "It is needed for websockets connection");
    }

    [Fact]
    public void EnableRosCommunication_ShouldParseTopicListToTheDesiredFormatAndSetItAsEnvVariableOfRelayNetApp()
    {
        //arrange
        var distro = RosDistro.Noetic;
        var depl = CreateExampleDeployment();
        var sut = new Ros1ConnectionBuilder(distro);
        var topicString = "[{\"topic_name\":\"/image_raw\",\"topic_type\":\"sensor_msgs/Image\"}]";
        var topics = new List<RosTopicModel>
        {
            new()
            {
                Name = TopicName.From("/image_raw"),
                Type = "sensor_msgs/Image",
                Description = "The example description that should not be included in the parsed topic",
                Enabled = true // this also should not be included
            }
        };
        //act
        var result = sut.EnableRosCommunication(depl, topics);

        //assert
        var relayNetAppContainer = result.Spec.Template.Spec.Containers.FirstOrDefault(c => c.Name == "relayNetApp");

        relayNetAppContainer.Should().NotBeNull();
        relayNetAppContainer!.Env.Should()
            .HaveCount(2, "We need ros_master_uri and list of topics for reading ros topics");

        var rosTopicsEnv = relayNetAppContainer.Env.FirstOrDefault(e => e.Name == "TOPIC_LIST");
        rosTopicsEnv.Should().NotBeNull();
        rosTopicsEnv!.Value.Should().Be(topicString);
    }

    [Fact]
    public void EnableRelayNetAppCommunication_ShouldAddWebsocketPortToExistingService()
    {
        //arrange
        var service = new V1Service
        {
            ApiVersion = "v1",
            Kind = "Service",
            Metadata = new()
            {
                Name = "example-service"
            },
            Spec = new()
            {
                Ports = new List<V1ServicePort>()
            }
        };
        var sut = new Ros1ConnectionBuilder(RosDistro.Noetic);
        //act
        var result = sut.EnableRelayNetAppCommunication(service);
        //assert

        result.Should().NotBeNull();
        result.Spec.Ports.Should()
            .HaveCountGreaterThan(0, "At least port 80 is needed for the Websocket communication");

        var port80 = result.Spec.Ports.FirstOrDefault(p => p.Port == 80);
        port80.Should().NotBeNull();
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