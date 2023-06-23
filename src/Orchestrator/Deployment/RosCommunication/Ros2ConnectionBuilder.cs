using k8s.Models;
using Middleware.Models.Enums;

namespace Middleware.Orchestrator.Deployment.RosCommunication;

internal class Ros2ConnectionBuilder : IRosConnectionBuilder
{
    private readonly RosDistro _distro;

    public Ros2ConnectionBuilder(RosDistro distro)
    {
        _distro = distro;
    }

    /// <inheritdoc />
    public V1Deployment EnableRosCommunication(V1Deployment dpl)
    {
        //TODO: we have to figure out how to enable ros2. It is possible that it will be same as ros1, but we will see :)
        return dpl;
    }
}