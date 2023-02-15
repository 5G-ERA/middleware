using MassTransit;
using Middleware.Common.MessageContracts;
using Middleware.Orchestrator.Deployment;
using Serilog;

namespace Middleware.Orchestrator.Handlers;

public class DeployPlanConsumer : IConsumer<DeployPlanMessage>
{
    private readonly IDeploymentService _deploymentService;
    private readonly Serilog.ILogger _logger;

    public DeployPlanConsumer(IDeploymentService deploymentService, Serilog.ILogger logger)
    {
        _deploymentService = deploymentService;
        _logger = logger;
    }

    public async Task Consume(ConsumeContext<DeployPlanMessage> ctx)
    {
        var plan = ctx.Message;
        var _ = await _deploymentService.DeployAsync(plan.Task, plan.RobotId);
    }
}