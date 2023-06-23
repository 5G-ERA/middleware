using Middleware.Models.Enums;

namespace Middleware.Orchestrator.Deployment;

internal interface IRosConnectionBuilderFactory
{
    IRosConnectionBuilder CreateConnectionBuilder(RosDistro distro);
}