using Middleware.Models.Domain;

namespace Middleware.Orchestrator.Deployment;

internal interface IRosConnectionBuilderFactory
{
    Task<IRosConnectionBuilder> CreateConnectionBuilder(RosDistro distro);
}