using Middleware.Models.Domain;

namespace Middleware.Orchestrator.Deployment;

internal interface IRosConnectionBuilderFactory
{
    IRosConnectionBuilder CreateConnectionBuilder(RosDistro distro);
}