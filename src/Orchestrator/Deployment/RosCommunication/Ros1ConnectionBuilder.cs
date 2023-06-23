using k8s.Models;
using Middleware.Models.Enums;

namespace Middleware.Orchestrator.Deployment.RosCommunication;

internal class Ros1ConnectionBuilder : IRosConnectionBuilder
{
    private readonly RosDistro _distro;

    public Ros1ConnectionBuilder(RosDistro distro)
    {
        _distro = distro;
        RosVersion = (int)distro;
        RosDistro = distro.ToString();
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

        var imageName = $"ros:{_distro.ToString().ToLower()}-ros-core";

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
            cont.Env.Add(rosMasterEnv);
            cont.Env.Add(rosIpEnv);
        }

        dpl.Spec.Template.Spec.Containers.Add(rosContainer);

        return dpl;
    }
}