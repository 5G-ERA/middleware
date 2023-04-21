using MassTransit;
using Microsoft.Extensions.Options;
using Middleware.Common.Config;
using Middleware.Common.MessageContracts;
using Middleware.Orchestrator.Deployment;

namespace Middleware.Orchestrator.Handlers.Switchover;

public class SwitchoverDeployInstanceConsumer : IConsumer<SwitchoverDeployAction>
{
    private readonly IDeploymentService _deploymentService;
    private readonly ILogger<SwitchoverDeployInstanceConsumer> _logger;
    private readonly IOptions<MiddlewareConfig> _mwConfig;

    public SwitchoverDeployInstanceConsumer(IDeploymentService deploymentService,
        ILogger<SwitchoverDeployInstanceConsumer> logger,
        IOptions<MiddlewareConfig> mwConfig)
    {
        _deploymentService = deploymentService;
        _logger = logger;
        _mwConfig = mwConfig;
    }

    public async Task Consume(ConsumeContext<SwitchoverDeployAction> context)
    {
        _logger.LogInformation("Started processing DeployPlanMessage");
        var payload = context.Message;
        var mwConfig = _mwConfig.Value;
        _logger.LogDebug("Location {0}-{1} received message request addressed to {2}", mwConfig.InstanceName,
            mwConfig.InstanceType, payload.Location);
        await _deploymentService.DeployActionAsync(payload.ActionPlanId, payload.ActionId);
    }
}