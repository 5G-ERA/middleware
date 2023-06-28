using k8s.Models;
using Middleware.Models.Domain;

namespace Middleware.Orchestrator.Deployment.RosCommunication;

internal class Ros2ConnectionBuilder : IRosConnectionBuilder
{
    private const RosVersion Ros2 = Middleware.Models.Domain.RosVersion.Ros2;
    private readonly RosDistro _distro;

    public Ros2ConnectionBuilder(RosDistro distro)
    {
        if (distro.RosVersion != Ros2)
        {
            throw new ArgumentException(
                "Ros1ConnectionBuilder cannot provide connectivity for ROs version other than 1", nameof(distro));
        }

        _distro = distro;
        RosVersion = distro.RosVersionInt;
        RosDistro = distro.Name;
    }


    /// <inheritdoc />
    public int RosVersion { get; }

    /// <inheritdoc />
    public string RosDistro { get; }

    /// <inheritdoc />
    public V1Deployment EnableRosCommunication(V1Deployment dpl)
    {
        //TODO: we have to figure out how to enable ros2. It is possible that it will be same as ros1, but we will see :)
        return dpl;
    }
}