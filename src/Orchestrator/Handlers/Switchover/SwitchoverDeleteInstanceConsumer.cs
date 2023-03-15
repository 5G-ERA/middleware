using MassTransit;
using Middleware.Common.MessageContracts;
using Middleware.Orchestrator.Deployment;

namespace Middleware.Orchestrator.Handlers.Switchover;

public class SwitchoverDeleteInstanceConsumer : IConsumer<SwitchoverDeleteInstance>
{
    private readonly IDeploymentService _deploymentService;
    public SwitchoverDeleteInstanceConsumer(IDeploymentService deploymentService)
    {
        _deploymentService = deploymentService;
    }

    public async Task Consume(ConsumeContext<SwitchoverDeleteInstance> context)
    {
        var payload = context.Message;
        //TODO: add message validation
        throw new NotImplementedException();
    }
}