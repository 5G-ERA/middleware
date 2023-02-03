using MassTransit;
using Middleware.Common.MessageContracts;
using Middleware.Orchestrator.Deployment;

namespace Middleware.Orchestrator.Handlers;

public class DeployPlanConsumer : IConsumer<DeployPlanMessage>
{
    private readonly IDeploymentService _deploymentService;

    public DeployPlanConsumer(IDeploymentService deploymentService)
    {
        _deploymentService = deploymentService;
    }

    public async Task Consume(ConsumeContext<DeployPlanMessage> ctx)
    {
        //TODO: additional handlers
        var plan = ctx.Message ?? throw new ArgumentNullException("ctx.Message");

        var result = await _deploymentService.DeployAsync(plan.Task, plan.RobotId);
    }
}