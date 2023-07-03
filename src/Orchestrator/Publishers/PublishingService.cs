using Middleware.Common;
using Middleware.Common.Helpers;
using Middleware.Common.MessageContracts;
using Middleware.Models.Domain.Contracts;

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

    public async Task PublishGatewayDeleteNetAppEntryAsync(ILocation desiredLocation, string netAppName,
        Guid actionPlanId,
        Guid serviceInstanceId)
    {
        var routingKey = QueueHelpers.ConstructRoutingKey(desiredLocation.Name, desiredLocation.Type);
        var message = new GatewayDeleteNetAppEntryMessage
        {
            NetAppName = netAppName,
            ActionPlanId = actionPlanId,
            ServiceInstanceId = serviceInstanceId,
            DeploymentLocation = routingKey
        };
        await _deleteNetAppEntryPublisher.PublishAsync(message);
    }

    public async Task PublishGatewayAddNetAppEntryAsync(ILocation desiredLocation, string netAppName, Guid actionPlanId,
        Guid serviceInstanceId)
    {
        var location = QueueHelpers.ConstructRoutingKey(desiredLocation.Name, desiredLocation.Type);
        var message = new GatewayAddNetAppEntryMessage
        {
            NetAppName = netAppName,
            ActionPlanId = actionPlanId,
            ServiceInstanceId = serviceInstanceId,
            DeploymentLocation = location,
            Route = netAppName
        };
        await _addNetAppEntryPublisher.PublishAsync(message);
    }
}