using System;
using System.Collections.Generic;
using System.Linq;
using FluentAssertions;
using Middleware.Models.Domain;
using Middleware.Models.Domain.Ros;
using Middleware.Orchestrator.Deployment.RosCommunication;
using Xunit;

namespace Orchestrator.Tests.Unit.Deployment;

public class Ros2ConnectionBuilderTests
{
    [Fact]
    public void Ros2ConnectionBuilder_ShouldThrowArgumentExceptionWhenGivenRos2Distro()
    {
        //arrange
        var ros2Distro = RosDistro.Noetic;
        var cfg = new SystemConfigModel();
        //act
        var act = () => new Ros2ConnectionBuilder(ros2Distro, cfg);
        //assert
        act.Should().Throw<ArgumentException>()
            .WithMessage(
                "Ros2ConnectionBuilder cannot provide connectivity for ROS version other than 2 (Parameter 'distro')");
    }
    
    [Fact]
    public void EnableRosCommunication_ShouldConfigureDeploymentWithNewContainers()
    {
        //arrange
        var distro = RosDistro.Foxy;
        var depl = K8SBuilder.CreateExampleDeployment();
        var cfg = new SystemConfigModel
        {
            Ros2RelayContainer = "but5gera/ros2_relay_server:0.2.0"
        };

        var sut = new Ros2ConnectionBuilder(distro, cfg);
        var topics = new List<RosTopicModel>();
        var rosSpec = new RosSpec(topics, topics, null, null, null);
        //act
        var result = sut.EnableRosCommunication(depl, rosSpec);
        //assert
        result.Spec.Template.Spec.Containers.Should()
            .HaveCount(2, "We need two containers, one RelayNetApp and NetApp itself");
        
        var relayNetAppContainer = result.Spec.Template.Spec.Containers.FirstOrDefault(c => c.Name == "relay-net-app");

        relayNetAppContainer.Should().NotBeNull();
        relayNetAppContainer!.Image.Should().Be(cfg.Ros2RelayContainer);
        relayNetAppContainer.Ports.Should().HaveCount(1);
        relayNetAppContainer.Ports.First().ContainerPort.Should().Be(80, "It is needed for websockets connection");
    }
    [Fact]
    public void EnableRosCommunication_ShouldParseTopicListToTheDesiredFormatAndSetItAsEnvVariableOfRelayNetApp()
    {
        //arrange
        var distro = RosDistro.Foxy;
        var cfg = new SystemConfigModel
        {
            Ros2RelayContainer = "but5gera/ros2_relay_server:0.2.0"
        };
        var depl = K8SBuilder.CreateExampleDeployment();
        var sut = new Ros2ConnectionBuilder(distro, cfg);
        var topicString = "[{\"name\":\"/image_raw\",\"type\":\"sensor_msgs/Image\",\"compression\":\"none\",\"qos\":null}]";
        var topics = new List<RosTopicModel>
        {
            new()
            {
                Name = "/image_raw",
                Type = "sensor_msgs/Image",
                Description = "The example description that should not be included in the parsed topic",
                Enabled = true // this also should not be included
            }
        };
        var rosSpec = new RosSpec(topics, topics, null, null, null);
        //act
        var result = sut.EnableRosCommunication(depl, rosSpec);

        //assert
        var relayNetAppContainer = result.Spec.Template.Spec.Containers.FirstOrDefault(c => c.Name == "relay-net-app");

        relayNetAppContainer.Should().NotBeNull();
        relayNetAppContainer!.Env.Should()
            .HaveCount(8,
                "We need a list of topics for reading ros topics, services, actions and transforms, and to specify the port NetApp will operate on");

        var rosTopicsEnv = relayNetAppContainer.Env.FirstOrDefault(e => e.Name == "TOPIC_LIST");
        rosTopicsEnv.Should().NotBeNull();
        rosTopicsEnv!.Value.Should().Be(topicString);
        
        var rosTopicsToClient = relayNetAppContainer.Env.FirstOrDefault(e => e.Name == "TOPICS_TO_CLIENT");
        rosTopicsToClient.Should().NotBeNull();
        rosTopicsToClient!.Value.Should().Be(topicString);

        rosTopicsToClient.Value.Should().BeEquivalentTo(rosTopicsEnv.Value);

        var netAppPortEnv = relayNetAppContainer.Env.FirstOrDefault(e => e.Name == "NETAPP_PORT");
        netAppPortEnv.Should().NotBeNull();
        netAppPortEnv!.Value.Should().Be("80", "For easy operation, NetApp has to work on port 80");
    }
    [Fact]
    public void EnableRosCommunication_ShouldAddServicesEnvVariable_WhenListOfServicesIsPassed()
    {
        //arrange
        var distro = RosDistro.Foxy;
        var cfg = new SystemConfigModel
        {
            Ros2RelayContainer = "but5gera/ros2_relay_server:0.2.0"
        };
        var depl = K8SBuilder.CreateExampleDeployment();
        var sut = new Ros2ConnectionBuilder(distro, cfg);
        var serviceStringWithoutQos = "[{\"name\":\"/test_srvs\",\"type\":\"/std_srvs/SetBool\",\"qos\":null}]";
        var topics = new List<RosTopicModel>()
        {
            new()
            {
                Name = "/test_srvs",
                Type = "/std_srvs/SetBool",
                Description = "The example description that should not be included in the parsed topic",
            }
        };
        var services = new List<RosServiceModel>
        {
            new()
            {
                Name = "/test_srvs",
                Type = "/std_srvs/SetBool",
                Description = "The example description that should not be included in the parsed topic",
            }
        };
        var rosSpec = new RosSpec(topics, topics, services, null, null);
        //act
        var result = sut.EnableRosCommunication(depl, rosSpec);

        //assert
        var relayNetAppContainer = result.Spec.Template.Spec.Containers.FirstOrDefault(c => c.Name == "relay-net-app");

        relayNetAppContainer.Should().NotBeNull();
        relayNetAppContainer!.Env.Should()
            .HaveCount(8,
                "We need a list of topics for reading ros topics, services, actions and transforms, and to specify the port NetApp will operate on");

        var rosTopicsEnv = relayNetAppContainer.Env.FirstOrDefault(e => e.Name == "SERVICES_FROM_CLIENT");
        rosTopicsEnv.Should().NotBeNull();
        rosTopicsEnv!.Value.Should().Be(serviceStringWithoutQos);
    }
    [Fact]
    public void EnableRosCommunication_ShouldAddServicesEnvVariable_WhenListOfServicesWithQosIsPassed()
    {
        //arrange
        var distro = RosDistro.Foxy;
        var cfg = new SystemConfigModel
        {
            Ros2RelayContainer = "but5gera/ros2_relay_server:0.2.0"
        };
        var depl = K8SBuilder.CreateExampleDeployment();
        var sut = new Ros2ConnectionBuilder(distro, cfg);
        var serviceStringWithQos = "[{\"name\":\"/test_srvs\",\"type\":\"/std_srvs/SetBool\"," +
                "\"qos\":{\"preset\":\"string\",\"history\":\"string\",\"depth\":12,\"reliability\":null," +
                "\"durability\":null,\"deadline\":null,\"lifespan\":null,\"liveliness\":null,\"lease\":\"string\"}}]";
        var topics = new List<RosTopicModel>()
        {
            new()
            {
                Name = "/test_srvs",
                Type = "/std_srvs/SetBool",
                Description = "The example description that should not be included in the parsed topic",
            }
        };
        var services = new List<RosServiceModel>
        {
            new()
            {
                Name = "/test_srvs",
                Type = "/std_srvs/SetBool",
                Description = "The example description that should not be included in the parsed topic",
                Qos = new Qos()
                {
                    Preset = "string",
                    Depth = 12,
                    Lease = "string",
                    History = "string"
                }
            }
        };
        var rosSpec = new RosSpec(topics, topics, services, null, null);
        //act
        var result = sut.EnableRosCommunication(depl, rosSpec);

        //assert
        var relayNetAppContainer = result.Spec.Template.Spec.Containers.FirstOrDefault(c => c.Name == "relay-net-app");

        relayNetAppContainer.Should().NotBeNull();
        relayNetAppContainer!.Env.Should()
            .HaveCount(8,
                "We need a list of topics for reading ros topics, services, actions and transforms, and to specify the port NetApp will operate on");

        var rosTopicsEnv = relayNetAppContainer.Env.FirstOrDefault(e => e.Name == "SERVICES_FROM_CLIENT");
        rosTopicsEnv.Should().NotBeNull();
        rosTopicsEnv!.Value.Should().Be(serviceStringWithQos);
    }
    [Fact]
    public void EnableRosCommunication_ShouldAddSActionsEnvVariable_WhenListOfActionsIsPassed()
    {
        //arrange
        var distro = RosDistro.Foxy;
        var cfg = new SystemConfigModel
        {
            Ros2RelayContainer = "but5gera/ros2_relay_server:0.2.0"
        };
        var depl = K8SBuilder.CreateExampleDeployment();
        var sut = new Ros2ConnectionBuilder(distro, cfg);
        var actionsString = "[{\"name\":\"/test_srvs\",\"type\":\"/std_srvs/SetBool\"}]";
        var topics = new List<RosTopicModel>()
        {
            new()
            {
                Name = "/test_srvs",
                Type = "/std_srvs/SetBool",
                Description = "The example description that should not be included in the parsed topic",
            }
        };
        var actions = new List<RosActionModel>
        {
            new()
            {
                Name = "/test_srvs",
                Type = "/std_srvs/SetBool",
            }
        };
        var rosSpec = new RosSpec(topics, topics, null, null, actions);
        //act
        var result = sut.EnableRosCommunication(depl, rosSpec);

        //assert
        var relayNetAppContainer = result.Spec.Template.Spec.Containers.FirstOrDefault(c => c.Name == "relay-net-app");

        relayNetAppContainer.Should().NotBeNull();
        relayNetAppContainer!.Env.Should()
            .HaveCount(8,
                "We need a list of topics for reading ros topics, services, actions and transforms, and to specify the port NetApp will operate on");

        var rosTopicsEnv = relayNetAppContainer.Env.FirstOrDefault(e => e.Name == "ACTIONS_FROM_CLIENT");
        rosTopicsEnv.Should().NotBeNull();
        rosTopicsEnv!.Value.Should().Be(actionsString);
    }
    
    [Fact]
    public void EnableRosCommunication_ShouldAddSTransformsEnvVariable_WhenListOfTransformsIsPassed()
    {
        //arrange
        var distro = RosDistro.Foxy;
        var cfg = new SystemConfigModel
        {
            Ros2RelayContainer = "but5gera/ros2_relay_server:0.2.0"
        };
        var depl = K8SBuilder.CreateExampleDeployment();
        var sut = new Ros2ConnectionBuilder(distro, cfg);
        var transformsString = "[{\"source_frame\":\"source\",\"target_frame\":\"target\",\"angular_thres\":0.1,\"trans_thres\":0.001,\"max_publish_period\":0.01}]";
        var topics = new List<RosTopicModel>()
        {
            new()
            {
                Name = "/test_srvs",
                Type = "/std_srvs/SetBool",
                Description = "The example description that should not be included in the parsed topic",
            }
        };
        var transforms = new List<RosTransformsModel>
        {
            new()
            {
                SourceFrame = "source",
                TargetFrame = "target",
                AngularThres = 0.1,
                TransThres = 0.001,
                MaxPublishPeriod = 0.01
            }
        };
        var rosSpec = new RosSpec(topics, topics, null, transforms, null);
        //act
        var result = sut.EnableRosCommunication(depl, rosSpec);

        //assert
        var relayNetAppContainer = result.Spec.Template.Spec.Containers.FirstOrDefault(c => c.Name == "relay-net-app");

        relayNetAppContainer.Should().NotBeNull();
        relayNetAppContainer!.Env.Should()
            .HaveCount(8,
                "We need a list of topics for reading ros topics, services, actions and transforms, and to specify the port NetApp will operate on");

        var rosTopicsEnv = relayNetAppContainer.Env.FirstOrDefault(e => e.Name == "TRANSFORMS_TO_CLIENT");
        rosTopicsEnv.Should().NotBeNull();
        rosTopicsEnv!.Value.Should().Be(transformsString);
    }
}