using System.Text.Json;
using k8s.Models;
using Middleware.Common.ExtensionMethods;
using Middleware.Models.Domain;

namespace Middleware.Orchestrator.Deployment.RosCommunication;

internal class Ros2ConnectionBuilder : IRosConnectionBuilder
{
    private const RosVersion Ros2 = Middleware.Models.Domain.RosVersion.Ros2;
    private readonly SystemConfigModel _cfg;
    private readonly RosDistro _distro;

    public Ros2ConnectionBuilder(RosDistro distro, SystemConfigModel cfg)
    {
        if (distro.RosVersion != Ros2)
        {
            throw new ArgumentException(
                "Ros1ConnectionBuilder cannot provide connectivity for ROs version other than 1", nameof(distro));
        }

        _distro = distro;
        _cfg = cfg;
        RosVersion = distro.RosVersionInt;
        RosDistro = distro.Name;
    }

    /// <inheritdoc />
    public int RosVersion { get; }

    /// <inheritdoc />
    public string RosDistro { get; }

    /// <inheritdoc />
    public V1Deployment EnableRosCommunication(V1Deployment dpl, IReadOnlyList<RosTopicModel> topicSubscribers,
        IReadOnlyList<RosTopicModel> topicPublishers)
    {
        if (topicSubscribers is null) throw new ArgumentNullException(nameof(topicSubscribers));
        if (dpl.Spec?.Template?.Spec?.Containers is null || dpl.Spec.Template.Spec.Containers.Any() == false)
            throw new ArgumentException("Missing Deployment container configuration.", nameof(dpl));

        dpl.Spec.Template.Spec.Containers.Add(GetRelayNetAppContainer(topicSubscribers, topicPublishers));

        return dpl;
    }

    /// <inheritdoc />
    public V1Service EnableRelayNetAppCommunication(V1Service service)
    {
        if (service is null) throw new ArgumentNullException(nameof(service));
        if (service.Spec?.Ports is null) throw new ArgumentNullException(nameof(service), "Ports list cannot be null");

        if (service.ContainsWebsocketCompatiblePort() == false)
            service.AddWebsocketCompatiblePort();

        return service;
    }

    private V1Container GetRelayNetAppContainer(IReadOnlyList<RosTopicModel> topicSubscribers,
        IReadOnlyList<RosTopicModel> topicPublishers)
    {
        var subscribersContainers = topicSubscribers.Select(RosTopicContainer.FromRosTopicModel).ToList();
        var publishersContainers = topicPublishers.Select(RosTopicContainer.FromRosTopicModel).ToList();
        var subscribersString = JsonSerializer.Serialize(subscribersContainers);
        var publishersString = JsonSerializer.Serialize(publishersContainers);
        var container = new V1Container
        {
            Name = "relay-net-app",
            Ports = new List<V1ContainerPort>
            {
                new(80, name: "websocket")
            },
            Env = new List<V1EnvVar>
            {
                new("TOPIC_LIST",
                    publishersString), // MK 2023.10.20: TOPIC_LIST - list of topics to be sent FROM cloud TO robot
                new("TOPIC_TO_PUB_LIST",
                    subscribersString), // MK 2023.10.20: TOPIC_TO_PUB_LIST - list of topics to be sent FROM robot TO cloud
                new("NETAPP_PORT", "80")
            },
            Image = _cfg.Ros2RelayContainer
        };

        return container;
    }
}