using Middleware.Common.MessageContracts;
using MassTransit;
using Middleware.OcelotGateway.Services;
using Middleware.Common.Config;
using System.Numerics;

namespace Middleware.OcelotGateway.Handlers;

public class CreateDynamicRouteConsumer : IConsumer<GatewayAddNetAppEntryMessage>
{
    private readonly ILogger _logger;

    private GatewayConfigurationService _gatewayConfigurationService;

    private readonly IConfiguration _cfg;

    public CreateDynamicRouteConsumer(ILogger<CreateDynamicRouteConsumer> logger, GatewayConfigurationService gatewayConfigurationService, IConfiguration cfg)
    {
        _logger = logger;
        _gatewayConfigurationService = gatewayConfigurationService;
        _cfg = cfg;

    }


    public Task Consume(ConsumeContext<GatewayAddNetAppEntryMessage> context)
    {
        _logger.LogInformation("Started processing GatewayAddNetAppEntryMessage");
        var mwconfig = _cfg.GetSection(MiddlewareConfig.ConfigName).Get<MiddlewareConfig>();
        var msg = context.Message;
        _logger.LogDebug("Location {0}-{1} received message request addressed to {2}", mwconfig.InstanceName, mwconfig.InstanceType, msg.DeploymentLocation);
        _gatewayConfigurationService.CreateDynamicRoute(msg);
        return Task.CompletedTask;
    }
}
