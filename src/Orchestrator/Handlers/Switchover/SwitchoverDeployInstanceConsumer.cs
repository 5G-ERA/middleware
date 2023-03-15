using MassTransit;
using Middleware.Common.MessageContracts;
using Middleware.Orchestrator.Deployment;

namespace Middleware.Orchestrator.Handlers.Switchover;

public class SwitchoverDeployInstanceConsumer : IConsumer<SwitchoverDeployInstance>
{
    private readonly IDeploymentService _deploymentService;

    public SwitchoverDeployInstanceConsumer(IDeploymentService deploymentService)
    {
        _deploymentService = deploymentService;
    }

    public async Task Consume(ConsumeContext<SwitchoverDeployInstance> context)
    {
        throw new NotImplementedException();
    }
}