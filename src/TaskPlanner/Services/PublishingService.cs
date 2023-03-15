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
    private readonly IPublisher<SwitchoverDeleteInstance> _switchoverDeleteInstancePublisher;
    private readonly IPublisher<SwitchoverDeployInstance> _switchoverDeployInstancePublisher;
    private readonly ICentralApiClient _centralApiClient;
    private readonly MiddlewareConfig _middlewareConfig;

    public PublishingService(IPublisher<DeployPlanMessage> deployPlanPublisher,
        IPublisher<SwitchoverDeleteInstance> switchoverDeleteInstancePublisher,
        IPublisher<SwitchoverDeployInstance> switchoverDeployInstancePublisher,
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

    public async Task PublishSwitchoverDeleteInstance(Guid actionPlanId, Guid instanceId)
    {
        var loc = QueueHelpers.ConstructRoutingKey(_middlewareConfig.InstanceName, _middlewareConfig.InstanceType);

        var payload = new SwitchoverDeleteInstance()
        {
            Location = loc,
            InstanceId = instanceId,
            ActionPlanId = actionPlanId
        };

        await _switchoverDeleteInstancePublisher.PublishAsync(payload);
    }

    public async Task PublishSwitchoverDeployInstance(Guid actionPlanId, Guid instanceId, string location, string locationType)
    {
        var response = await _centralApiClient.GetAvailableLocations();
        if (response is null || !response.Locations.Any(l => l.Name == location && l.Type == locationType))
        {
            throw new IncorrectLocationException(location, locationType);
        }

        var loc = QueueHelpers.ConstructRoutingKey(location, locationType);

        var payload = new SwitchoverDeployInstance()
        {
            Location = loc,
            InstanceId = instanceId,
            ActionPlanId = actionPlanId
        };

        await _switchoverDeployInstancePublisher.PublishAsync(payload);
    }
}