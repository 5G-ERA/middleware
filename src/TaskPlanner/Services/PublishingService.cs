using Microsoft.Extensions.Options;
using Middleware.CentralApi.Sdk;
using Middleware.Common.Config;
using Middleware.Common.Helpers;
using Middleware.Common.MessageContracts;
using Middleware.Models.Domain;
using Middleware.TaskPlanner.Exceptions;
using Middleware.TaskPlanner.Publishers;

namespace Middleware.TaskPlanner.Services;

public class PublishingService : IPublishService
{
    private readonly IPublisher<DeployPlanMessage> _deployPlanPublisher;
    private readonly IPublisher<SwitchoverDeleteAction> _switchoverDeleteInstancePublisher;
    private readonly IPublisher<SwitchoverDeployAction> _switchoverDeployInstancePublisher;
    private readonly ICentralApiClient _centralApiClient;
    private readonly MiddlewareConfig _middlewareConfig;

    public PublishingService(IPublisher<DeployPlanMessage> deployPlanPublisher,
        IPublisher<SwitchoverDeleteAction> switchoverDeleteInstancePublisher,
        IPublisher<SwitchoverDeployAction> switchoverDeployInstancePublisher,
        IOptions<MiddlewareConfig> middlewareConfig,
        ICentralApiClient centralApiClient)
    {
        _deployPlanPublisher = deployPlanPublisher;
        _switchoverDeleteInstancePublisher = switchoverDeleteInstancePublisher;
        _switchoverDeployInstancePublisher = switchoverDeployInstancePublisher;
        _centralApiClient = centralApiClient;
        _middlewareConfig = middlewareConfig.Value;
    }

    public async Task PublishPlanAsync(TaskModel task, RobotModel robot)
    {
        var action = task.ActionSequence!.FirstOrDefault();

        if (action == null)
            return;

        var location = QueueHelpers.ConstructRoutingKey(action.Placement, action.PlacementType);
        var message = new DeployPlanMessage()
        {
            Task = task,
            RobotId = robot.Id,
            DeploymentLocation = location
        };
        await _deployPlanPublisher.PublishAsync(message);
    }

    public async Task PublishSwitchoverDeleteInstance(Guid actionPlanId, Guid actionId)
    {
        var loc = QueueHelpers.ConstructRoutingKey(_middlewareConfig.InstanceName, _middlewareConfig.InstanceType);

        var payload = new SwitchoverDeleteAction()
        {
            Location = loc,
            ActionId = actionId,
            ActionPlanId = actionPlanId
        };

        await _switchoverDeleteInstancePublisher.PublishAsync(payload);
    }

    public async Task PublishSwitchoverDeployInstance(Guid actionPlanId, Guid actionId, string location, string locationType)
    {
        var response = await _centralApiClient.GetAvailableLocations();
        if (response is null || !response.Locations.Any(l => l.Name == location && l.Type == locationType))
        {
            throw new IncorrectLocationException(location, locationType);
        }

        var loc = QueueHelpers.ConstructRoutingKey(location, locationType);

        var payload = new SwitchoverDeployAction()
        {
            Location = loc,
            ActionId = actionId,
            ActionPlanId = actionPlanId
        };

        await _switchoverDeployInstancePublisher.PublishAsync(payload);
    }
}