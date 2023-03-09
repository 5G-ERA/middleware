using MassTransit;
using Middleware.Common.Config;
using Middleware.Common.MessageContracts;
using Middleware.Orchestrator.Deployment;


namespace Middleware.Orchestrator.Handlers;

public class DeployPlanConsumer : IConsumer<DeployPlanMessage>
{
    private readonly IDeploymentService _deploymentService;
    private readonly ILogger _logger;
    private readonly IConfiguration _cfg;

    public DeployPlanConsumer(IDeploymentService deploymentService, ILogger<DeployPlanConsumer> logger, IConfiguration cfg)
    {
        _deploymentService = deploymentService;
        _logger = logger;
        _cfg = cfg;
    }

    public async Task Consume(ConsumeContext<DeployPlanMessage> ctx)
    {
        _logger.LogInformation("Started processing DeployPlanMessage");
        var mwconfig = _cfg.GetSection(MiddlewareConfig.ConfigName).Get<MiddlewareConfig>();
        var plan = ctx.Message;
        _logger.LogDebug("Location {0}-{1} received message request addressed to {2}", mwconfig.InstanceName, mwconfig.InstanceType, plan.DeploymentLocation);
        var _ = await _deploymentService.DeployAsync(plan.Task, plan.RobotId);
    }
}