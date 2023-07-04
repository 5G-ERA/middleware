using Middleware.Models.Domain.Contracts;

namespace Middleware.Orchestrator.Publishers;

internal interface IPublishingService
{
    Task PublishGatewayAddNetAppEntryAsync(ILocation desiredLocation, string netAppName, Guid actionPlanId,
        Guid serviceInstanceId);

    Task PublishGatewayDeleteNetAppEntryAsync(ILocation desiredLocation, string netAppName, Guid actionPlanId,
        Guid serviceInstanceId);
}