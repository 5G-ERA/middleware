using MassTransit;
using Microsoft.Extensions.Options;
using Middleware.Common.Config;
using Middleware.Common.MessageContracts;
using Middleware.Orchestrator.Deployment;

namespace Middleware.Orchestrator.Handlers.Switchover;

public class SwitchoverDeleteInstanceConsumer : IConsumer<SwitchoverDeleteAction>
{
    private readonly IDeploymentService _deploymentService;
    private readonly ILogger<SwitchoverDeleteInstanceConsumer> _logger;
    private readonly IOptions<MiddlewareConfig> _mwConfig;

    public SwitchoverDeleteInstanceConsumer(IDeploymentService deploymentService,
        ILogger<SwitchoverDeleteInstanceConsumer> logger, IOptions<MiddlewareConfig> mwConfig)
    {
        _deploymentService = deploymentService;
        _logger = logger;
        _mwConfig = mwConfig;
    }

    public async Task Consume(ConsumeContext<SwitchoverDeleteAction> context)
    {
        _logger.LogInformation("Started processing DeployPlanMessage");
        var payload = context.Message;
        var mwConfig = _mwConfig.Value;
        _logger.LogDebug("Location {0}-{1} received message request addressed to {2}", mwConfig.InstanceName,
            mwConfig.InstanceType, payload.Location);
        await _deploymentService.DeleteActionAsync(payload.ActionPlanId, payload.ActionId);
    }
}