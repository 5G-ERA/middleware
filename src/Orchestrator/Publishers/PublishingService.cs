using Middleware.Common;
using Middleware.Common.Helpers;
using Middleware.Common.MessageContracts;
using Middleware.Models.Domain;

namespace Middleware.Orchestrator.Publishers;

internal class PublishingService : IPublishingService
{
    private readonly IPublisher<GatewayAddNetAppEntryMessage> _addNetAppEntryPublisher;
    private readonly IPublisher<GatewayDeleteNetAppEntryMessage> _deleteNetAppEntryPublisher;

    public PublishingService(IPublisher<GatewayAddNetAppEntryMessage> addNetAppEntryPublisher,
        IPublisher<GatewayDeleteNetAppEntryMessage> deleteNetAppEntryPublisher)
    {
        _addNetAppEntryPublisher = addNetAppEntryPublisher;
        _deleteNetAppEntryPublisher = deleteNetAppEntryPublisher;
    }

    public async Task PublishGatewayAddNetAppEntryAsync(ActionModel action, string netAppName, Guid actionPlanId,
        Guid serviceInstanceId)
    {
        var location = QueueHelpers.ConstructRoutingKey(action.Placement, action.PlacementType);
        var message = new GatewayAddNetAppEntryMessage
        {
            NetAppName = netAppName,
            ActionPlanId = actionPlanId,
            ServiceInstanceId = serviceInstanceId,
            DeploymentLocation = location
        };
        await _addNetAppEntryPublisher.PublishAsync(message);
    }

    public async Task PublishGatewayDeleteNetAppEntryAsync(ActionModel action, string netAppName, Guid actionPlanId,
        Guid serviceInstanceId)
    {
        var location = QueueHelpers.ConstructRoutingKey(action.Placement, action.PlacementType);
        var message = new GatewayDeleteNetAppEntryMessage
        {
            NetAppName = netAppName,
            ActionPlanId = actionPlanId,
            ServiceInstanceId = serviceInstanceId,
            DeploymentLocation = location
        };
        await _deleteNetAppEntryPublisher.PublishAsync(message);
    }
}