using Middleware.Models.Domain;
using Middleware.Orchestrator.Deployment.RosCommunication;

namespace Middleware.Orchestrator.Deployment;

internal class RosConnectionBuilderFactory : IRosConnectionBuilderFactory
{
    /// <inheritdoc />
    public IRosConnectionBuilder CreateConnectionBuilder(RosDistro distro)
    {
        IRosConnectionBuilder retVal = distro.RosVersion switch
        {
            RosVersion.Ros1 => new Ros1ConnectionBuilder(distro),
            RosVersion.Ros2 => new Ros2ConnectionBuilder(distro),
            _ => null
        };

        return retVal;
    }
}