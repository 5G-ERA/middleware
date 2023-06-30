using MassTransit;
using Middleware.Common.Config;
using Middleware.Common.MessageContracts;
using Middleware.OcelotGateway.Services;

namespace Middleware.OcelotGateway.Handlers;

public class DeleteDynamicRouteConsumer : IConsumer<GatewayDeleteNetAppEntryMessage>
{
    private readonly ILogger _logger;

    private GatewayConfigurationService _gatewayConfigurationService;

    private readonly IConfiguration _cfg;

    public DeleteDynamicRouteConsumer(ILogger<DeleteDynamicRouteConsumer> logger, GatewayConfigurationService gatewayConfigurationService, IConfiguration cfg)
    {
        _logger = logger;
        _gatewayConfigurationService = gatewayConfigurationService;
        _cfg = cfg;
    }



    public Task Consume(ConsumeContext<GatewayDeleteNetAppEntryMessage> context)
    {
        _logger.LogInformation("Started processing GatewayDeleteNetAppEntryMessage");
        var mwconfig = _cfg.GetSection(MiddlewareConfig.ConfigName).Get<MiddlewareConfig>();
        var msg = context.Message;
        _logger.LogDebug("Location {0}-{1} received message request addressed to {2}", mwconfig.InstanceName, mwconfig.InstanceType, msg.DeploymentLocation);
        _gatewayConfigurationService.DeleteDynamicRoute(msg);
        return Task.CompletedTask;
    }
}
