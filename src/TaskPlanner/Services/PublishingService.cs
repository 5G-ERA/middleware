using Microsoft.Extensions.Options;
using Middleware.CentralApi.Sdk;
using Middleware.Common;
using Middleware.Common.Config;
using Middleware.Common.Helpers;
using Middleware.Common.MessageContracts;
using Middleware.Models.Domain;
using Middleware.TaskPlanner.Exceptions;

namespace Middleware.TaskPlanner.Services;

public class PublishingService : IPublishService
{
    private readonly ICentralApiClient _centralApiClient;
    private readonly IPublisher<ConnectRobotToSliceMessage> _connectRobotToSlicePublisher;
    private readonly IPublisher<DeployPlanMessage> _deployPlanPublisher;
    private readonly MiddlewareConfig _middlewareConfig;

    private readonly IRequestResponseClient<RequestResourcePlanMessage, RequestResourcePlanMessage>
        _requestResourcePlanClient;

    private readonly IPublisher<SwitchoverDeleteAction> _switchoverDeleteInstancePublisher;
    private readonly IPublisher<SwitchoverDeployAction> _switchoverDeployInstancePublisher;


    public PublishingService(IPublisher<DeployPlanMessage> deployPlanPublisher,
        IPublisher<SwitchoverDeleteAction> switchoverDeleteInstancePublisher,
        IPublisher<SwitchoverDeployAction> switchoverDeployInstancePublisher,
        IPublisher<ConnectRobotToSliceMessage> connectRobotToSlicePublisher,
        IOptions<MiddlewareConfig> middlewareConfig,
        ICentralApiClient centralApiClient,
        IRequestResponseClient<RequestResourcePlanMessage, RequestResourcePlanMessage> requestResourcePlanClient)
    {
        _deployPlanPublisher = deployPlanPublisher;
        _switchoverDeleteInstancePublisher = switchoverDeleteInstancePublisher;
        _switchoverDeployInstancePublisher = switchoverDeployInstancePublisher;
        _connectRobotToSlicePublisher = connectRobotToSlicePublisher;
        _centralApiClient = centralApiClient;
        _requestResourcePlanClient = requestResourcePlanClient;
        _middlewareConfig = middlewareConfig.Value;
    }

    public async Task PublishPlanAsync(TaskModel task, RobotModel robot)
    {
        var action = task.ActionSequence!.FirstOrDefault();

        if (action == null) return;

        var location = QueueHelpers.ConstructRoutingKey(action.Placement, action.PlacementType);
        var message = new DeployPlanMessage
        {
            Task = task,
            RobotId = robot.Id,
            DeploymentLocation = location
        };
        await _deployPlanPublisher.PublishAsync(message);

        foreach (var actionTmp in task.ActionSequence)
        {
            if (actionTmp.HasLocationWithNetWorkSliceSet() && string.IsNullOrEmpty(robot.SimCardNumber) == false)
            {
                await PublishConnectImsiToSlice(robot.Id, task.ActionPlanId, robot.SimCardNumber,
                    actionTmp.NetworkSlice,
                    actionTmp.Placement,
                    actionTmp.PlacementType);
            }
        }
    }

    public async Task PublishSwitchoverDeleteInstance(Guid actionPlanId, Guid actionId)
    {
        var loc = QueueHelpers.ConstructRoutingKey(_middlewareConfig.InstanceName, _middlewareConfig.InstanceType);

        var payload = new SwitchoverDeleteAction
        {
            Location = loc,
            ActionId = actionId,
            ActionPlanId = actionPlanId
        };

        await _switchoverDeleteInstancePublisher.PublishAsync(payload);
    }

    public async Task PublishSwitchoverDeployInstance(Guid actionPlanId, Guid actionId, string location,
        string locationType)
    {
        var response = await _centralApiClient.GetAvailableLocations();
        if (response is null || !response.Locations.Any(l => l.Name == location && l.Type == locationType))
            throw new IncorrectLocationException(location, locationType);

        var loc = QueueHelpers.ConstructRoutingKey(location, locationType);

        var payload = new SwitchoverDeployAction
        {
            Location = loc,
            ActionId = actionId,
            ActionPlanId = actionPlanId
        };

        await _switchoverDeployInstancePublisher.PublishAsync(payload);
    }

    public async Task PublishConnectImsiToSlice(Guid robotId, Guid actionPlanId, string imsi, string slice,
        string location,
        string locationType)
    {
        var response = await _centralApiClient.GetAvailableLocations();
        if (response is null || !response.Locations.Any(l => l.Name == location && l.Type == locationType))
            throw new IncorrectLocationException(location, locationType);


        var loc = QueueHelpers.ConstructRoutingKey(location, locationType);

        var payload = new ConnectRobotToSliceMessage
        {
            RobotId = robotId,
            ActionPlanId = actionPlanId,
            Imsi = imsi,
            Slice = slice,
            Location = loc
        };
        await _connectRobotToSlicePublisher.PublishAsync(payload);
    }

    /// <inheritdoc />
    public async Task<TaskModel> RequestResourcePlan(TaskModel task)
    {
        var payload = new RequestResourcePlanMessage
        {
            Task = task
        };
        var resp = await _requestResourcePlanClient.Request(payload);

        return resp.Task;
    }
}