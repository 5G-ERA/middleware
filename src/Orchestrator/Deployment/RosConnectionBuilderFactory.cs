using Middleware.Models.Enums;
using Middleware.Orchestrator.Deployment.RosCommunication;

namespace Middleware.Orchestrator.Deployment;

internal class RosConnectionBuilderFactory : IRosConnectionBuilderFactory
{
    /// <inheritdoc />
    public IRosConnectionBuilder CreateConnectionBuilder(RosDistro distro)
    {
        IRosConnectionBuilder retVal = null;
        switch ((int)distro)
        {
            case 1:
                retVal = new Ros1ConnectionBuilder(distro);
                break;
            case 2:
                retVal = new Ros2ConnectionBuilder(distro);
                break;
        }

        return retVal;
    }
}