using k8s.Models;
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
                "Ros1ConnectionBuilder cannot provide connectivity for ROs version other than 1", nameof(distro));
        }

        _distro = distro;
        RosVersion = distro.RosVersionInt;
        RosDistro = distro.Name;
    }

    public int RosVersion { get; }

    /// <inheritdoc />
    public string RosDistro { get; }

    /// <inheritdoc />
    /// <inheritdoc />
    public V1Deployment EnableRosCommunication(V1Deployment dpl)
    {
        if (dpl.Spec?.Template?.Spec?.Containers is null || dpl.Spec.Template.Spec.Containers.Any() == false)
            throw new ArgumentException("Missing Deployment container configuration.", nameof(dpl));

        var imageName = $"ros:{_distro.Name.ToLower()}-ros-core";

        var rosContainer = new V1Container
        {
            Image = imageName,
            Name = _distro.ToString(),
            Ports = new List<V1ContainerPort>(1)
            {
                new(11311, name: "ros-master")
            }
        };
        //BB 2023.06.23: containers within single pod communicate over loopback (localhost) interface 
        var rosMasterEnv = new V1EnvVar("ROS_MASTER_URI", "http://127.0.0.1:11311");
        var rosIpEnv = new V1EnvVar("ROS_IP", "127.0.0.1");
        foreach (var cont in dpl.Spec.Template.Spec.Containers)
        {
            cont.Env ??= new List<V1EnvVar>();
            cont.Env.Add(rosMasterEnv);
            cont.Env.Add(rosIpEnv);
        }

        dpl.Spec.Template.Spec.Containers.Add(rosContainer);

        return dpl;
    }
}