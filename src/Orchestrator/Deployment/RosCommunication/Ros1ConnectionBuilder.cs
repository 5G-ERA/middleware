using System.Text.Json;
using System.Text.Json.Serialization;
using JetBrains.Annotations;
using k8s.Models;
using Middleware.Common.ExtensionMethods;
using Middleware.Models.Domain;

namespace Middleware.Orchestrator.Deployment.RosCommunication;

internal class Ros1ConnectionBuilder : IRosConnectionBuilder
{
    private const RosVersion Ros1 = Middleware.Models.Domain.RosVersion.Ros1;
    private readonly RosDistro _distro;

    public Ros1ConnectionBuilder(RosDistro distro)
    {
        if (distro.RosVersion != Ros1)
        {
            throw new ArgumentException(
                "Ros1ConnectionBuilder cannot provide connectivity for ROS version other than 1", nameof(distro));
        }

        _distro = distro;
        RosVersion = distro.RosVersionInt;
        RosDistro = distro.Name;
    }

    public int RosVersion { get; }

    /// <inheritdoc />
    public string RosDistro { get; }

    public V1Service EnableRelayNetAppCommunication(V1Service service)
    {
        if (service is null) throw new ArgumentNullException(nameof(service));
        if (service.Spec?.Ports is null) throw new ArgumentNullException(nameof(service), "Ports list cannot be null");

        if (service.ContainsWebsocketCompatiblePort() == false)
            service.AddWebsocketCompatiblePort();

        return service;
    }

    /// <inheritdoc />
    public V1Deployment EnableRosCommunication(V1Deployment dpl, IReadOnlyList<RosTopicModel> rosTopics)
    {
        if (rosTopics is null) throw new ArgumentNullException(nameof(rosTopics));
        if (dpl.Spec?.Template?.Spec?.Containers is null || dpl.Spec.Template.Spec.Containers.Any() == false)
            throw new ArgumentException("Missing Deployment container configuration.", nameof(dpl));

        //BB 2023.06.23: containers within single pod communicate over loopback (localhost) interface 
        var rosMasterEnv = new V1EnvVar("ROS_MASTER_URI", "http://127.0.0.1:11311");
        var rosIpEnv = new V1EnvVar("ROS_IP", "127.0.0.1");
        foreach (var cont in dpl.Spec.Template.Spec.Containers)
        {
            cont.Env ??= new List<V1EnvVar>();
            cont.Env.Add(rosMasterEnv);
            cont.Env.Add(rosIpEnv);
        }

        dpl.Spec.Template.Spec.Containers.Add(GetRosContainer());
        dpl.Spec.Template.Spec.Containers.Add(GetRelayNetAppContainer(rosTopics));

        return dpl;
    }

    private V1Container GetRosContainer()
    {
        var rosContainer = new V1Container
        {
            Image = $"ros:{_distro.Name.ToLower()}-ros-core",
            Name = _distro.Name.ToLower(),
            Ports = new List<V1ContainerPort>
            {
                new(11311, name: "ros-master")
            },
            Args = new List<string> { "roscore" }
        };
        return rosContainer;
    }

    private V1Container GetRelayNetAppContainer(IReadOnlyList<RosTopicModel> rosTopics)
    {
        var rosTopicContainers = rosTopics.Select(RosTopicContainer.FromRosTopicModel).ToList();
        var topicsString = JsonSerializer.Serialize(rosTopicContainers);
        var container = new V1Container
        {
            Name = "relay-net-app",
            Ports = new List<V1ContainerPort>
            {
                new(80, name: "websocket")
            },
            Env = new List<V1EnvVar>
            {
                new("ROS_MASTER_URI", "http://127.0.0.1:11311"),
                new("TOPIC_LIST", topicsString),
                new("NETAPP_PORT", "80")
            },
            Image = "but5gera/relay_network_application:0.4.0"
        };

        return container;
    }

    /// <summary>
    ///     Temporary class to parse the RosTopicModel into the correct format
    /// </summary>
    private class RosTopicContainer
    {
        [JsonPropertyName("topic_name")]
        public string Name { [UsedImplicitly] get; init; }

        [JsonPropertyName("topic_type")]
        public string Type { [UsedImplicitly] get; init; }

        public static RosTopicContainer FromRosTopicModel(RosTopicModel topic)
        {
            return new()
            {
                Name = topic.Name,
                Type = topic.Type
            };
        }
    }
}