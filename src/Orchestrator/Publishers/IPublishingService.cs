using Middleware.Models.Domain;

namespace Middleware.Orchestrator.Publishers;

internal interface IPublishingService
{
    Task PublishGatewayAddNetAppEntryAsync(ActionModel action, string netAppName, Guid actionPlanId,
        Guid serviceInstanceId);

    Task PublishGatewayDeleteNetAppEntryAsync(ActionModel action, string netAppName, Guid actionPlanId,
        Guid serviceInstanceId);
}