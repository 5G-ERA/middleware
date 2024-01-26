using System.Text.Json;
using k8s.Models;
using Middleware.Common.ExtensionMethods;
using Middleware.Models.Domain;
using Middleware.Models.Domain.Ros;
using Middleware.Orchestrator.Deployment.RosCommunication.Structures;

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
                "Ros2ConnectionBuilder cannot provide connectivity for ROS version other than 2", nameof(distro));
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
    public V1Deployment EnableRosCommunication(V1Deployment dpl, RosSpec rosSpec)
    {
        if (rosSpec is null) throw new ArgumentNullException(nameof(rosSpec));
        if (dpl.Spec?.Template?.Spec?.Containers is null || dpl.Spec.Template.Spec.Containers.Any() == false)
            throw new ArgumentException("Missing Deployment container configuration.", nameof(dpl));

        dpl.Spec.Template.Spec.Containers.Add(GetRelayNetAppContainer(rosSpec));

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

    private V1Container GetRelayNetAppContainer(RosSpec rosSpec)
    {
        var subscribersContainers = rosSpec.TopicSubscribers.Select(RosTopicContainer.FromRosTopicModel).ToList();
        var publishersContainers = rosSpec.TopicPublishers.Select(RosTopicContainer.FromRosTopicModel).ToList();
        var serviceSubscribers = rosSpec.ServiceSubscribers?.Select(RosServiceContainer.FromRosServiceModel).ToList();
        var transformsPublishers =
            rosSpec.TransformsPublishers?.Select(RosTransformsContainer.FromRosTransformsModel).ToList();
        var actionSubscribers = rosSpec.ActionSubscribers?.Select(RosActionContainer.FromRosActionModel).ToList();
        var subscribersString = JsonSerializer.Serialize(subscribersContainers);
        var publishersString = JsonSerializer.Serialize(publishersContainers);
        var serviceSubscriberString = JsonSerializer.Serialize(serviceSubscribers);
        var transformsPublishersString = JsonSerializer.Serialize(transformsPublishers);
        var actionSubscribersString = JsonSerializer.Serialize(actionSubscribers);

        var container = new V1Container
        {
            Name = "relay-net-app",
            Ports = new List<V1ContainerPort>
            {
                new(80, name: "websocket")
            },
            Env = new List<V1EnvVar>
            {
                // legacy env variables before version 0.2.0
                new("TOPIC_LIST", publishersString),
                new("TOPIC_TO_PUB_LIST", subscribersString),
                // new env variables for version 0.2.0 and up 
                new("TOPICS_TO_CLIENT",
                    publishersString), // MK 2023.10.20: TOPIC_LIST - list of topics to be sent FROM cloud TO robot
                new("TOPICS_FROM_CLIENT",
                    subscribersString), // MK 2023.10.20: TOPIC_TO_PUB_LIST - list of topics to be sent FROM robot TO cloud
                new("SERVICES_FROM_CLIENT", serviceSubscriberString),
                new("TRANSFORMS_TO_CLIENT", transformsPublishersString),
                new("ACTIONS_FROM_CLIENT", actionSubscribersString),
                new("NETAPP_PORT", "80")
            },
            Image = _cfg.Ros2RelayContainer
        };

        return container;
    }
}