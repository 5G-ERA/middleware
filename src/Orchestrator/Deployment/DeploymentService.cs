using k8s.Autorest;
using Microsoft.Extensions.Options;
using Middleware.Common.Config;
using Middleware.Common.Enums;
using Middleware.Common.ExtensionMethods;
using Middleware.Common.Result;
using Middleware.DataAccess.Repositories.Abstract;
using Middleware.Models.Domain;
using Middleware.Models.Domain.Contracts;
using Middleware.Models.Enums;
using Middleware.Models.ExtensionMethods;
using Middleware.Orchestrator.Config;
using Middleware.Orchestrator.Deployment.RosCommunication;
using Middleware.Orchestrator.Exceptions;
using Middleware.Orchestrator.Models;
using Middleware.Orchestrator.Publishers;
using Middleware.RedisInterface.Contracts.Mappings;
using Middleware.RedisInterface.Sdk;

namespace Middleware.Orchestrator.Deployment;

internal class DeploymentService : IDeploymentService
{
    /// <summary>
    ///     Builds and configures the kubernetes objects
    /// </summary>
    private readonly IKubernetesObjectBuilder _kubeObjectBuilder;

    /// <summary>
    ///     Logger instance
    /// </summary>
    private readonly ILogger _logger;

    /// <summary>
    ///     Middleware configuration
    /// </summary>
    private readonly IOptions<MiddlewareConfig> _mwConfig;

    /// <summary>
    ///     Service that publishes RabbitMQ messages
    /// </summary>
    private readonly IPublishingService _publisher;

    /// <summary>
    ///     Redis Interface API client
    /// </summary>
    private readonly IRedisInterfaceClient _redisInterfaceClient;

    /// <summary>
    ///     Factory building ROS communication enablers
    /// </summary>
    private readonly IRosConnectionBuilderFactory _rosConnectionBuilderFactory;

    private readonly ISystemConfigRepository _systemConfigRepository;
    private readonly IKubernetesWrapper _kubernetesWrapper;

    public DeploymentService(ILogger<DeploymentService> logger,
        IRedisInterfaceClient redisInterfaceClient,
        IOptions<MiddlewareConfig> mwConfig,
        IKubernetesObjectBuilder kubeObjectBuilder, IRosConnectionBuilderFactory rosConnectionBuilderFactory,
        IPublishingService publisher, ISystemConfigRepository systemConfigRepository,
        IKubernetesWrapper kubernetesWrapper)
    {
        _logger = logger;
        _redisInterfaceClient = redisInterfaceClient;
        _mwConfig = mwConfig;
        _kubeObjectBuilder = kubeObjectBuilder;
        _rosConnectionBuilderFactory = rosConnectionBuilderFactory;
        _publisher = publisher;
        _systemConfigRepository = systemConfigRepository;
        _kubernetesWrapper = kubernetesWrapper;
    }

    /// <inheritdoc />
    public async Task<Result> DeletePlanAsync(ActionPlanModel actionPlan)
    {
        var retVal = new Result();
        try
        {
            foreach (var action in actionPlan.ActionSequence!)
            {
                var relayShouldBeDeleted = true;
                var actionLoc =
                    await GetLocationAsync(Enum.Parse<LocationType>(action.PlacementType!), action.Placement);

                foreach (var srv in action.Services)
                {
                    if (srv.IsReusable is not null && srv.IsReusable!.Value &&
                        await IsServiceUsedInOtherActionPlans(actionPlan.Id, srv.ServiceInstanceId))
                    {
                        relayShouldBeDeleted = false;
                        continue;
                    }

                    retVal += await _kubernetesWrapper.TerminateNetAppById(srv.ServiceInstanceId);
                    await _redisInterfaceClient.DeleteRelationAsync(srv, actionLoc.ToBaseLocation(),
                        "LOCATED_AT");

                    if (ShouldDeployInterRelay(srv, action) == false)
                    {
                        await _publisher.PublishGatewayDeleteNetAppEntryAsync(actionLoc, srv.Name, actionPlan.Id,
                            srv.ServiceInstanceId);
                    }
                }

                if (relayShouldBeDeleted && action.ShouldUseInterRelayForRosNetApps())
                {
                    var labels = _kubeObjectBuilder.CreateInterRelayNetAppLabels(actionPlan.Id, action.Id);
                    var deleteRelayResult = await _kubernetesWrapper.TerminateInterRelayNetApp(labels);
                    if (deleteRelayResult.IsSuccess)
                    {
                        await _publisher.PublishGatewayDeleteNetAppEntryAsync(actionLoc, deleteRelayResult.Value,
                            actionPlan.Id,
                            action.Id);
                    }
                }
            }
        }
        catch (NotInK8SEnvironmentException ex)
        {
            _logger.LogInformation("The instantiation of the kubernetes client has failed in {env} environment.",
                AppConfig.AppConfiguration);

            retVal = AppConfig.AppConfiguration == AppVersionEnum.Dev.GetStringValue()
                ? Result.Success()
                : Result.Failure(ex.Message);
            if (retVal.IsSuccess == false)
                _logger.LogWarning("Deployment of the services has been skipped in the Development environment");
        }
        catch (HttpOperationException ex)
        {
            _logger.LogError(ex, "There was an error while deleting the service caused by {reason}",
                ex.Response.Content);
            retVal = ex.Response.Content;
        }
        catch (Exception ex)
        {
            _logger.LogError(ex, "The deletion of the Action Plan has failed!");
            retVal = ex.Message;
        }

        return retVal;
    }

    public async Task DeleteActionAsync(Guid actionPlanId, Guid actionId)
    {
        _logger.LogTrace("Entered DeployActionAsync");
        _logger.LogDebug("Retrieving ActionPlan");
        var actionPlan = await _redisInterfaceClient.ActionPlanGetByIdAsync(actionPlanId);
        if (actionPlan is null)
            return;
        _logger.LogDebug("Retrieving Action from action plan");
        var action = actionPlan.ActionSequence!.FirstOrDefault(x => x.Id == actionId);
        if (action is null)
            return;

        _logger.LogDebug("Retrieving location details (cloud or edge)");
        var thisLocation = await GetCurrentLocationAsync();

        foreach (var instance in action.Services)
        {
            _logger.LogDebug("Deleting instance '{instanceName}', with serviceInstanceId '{serviceInstanceId}'",
                instance.Name, instance.ServiceInstanceId);

            await _kubernetesWrapper.TerminateNetAppById(instance.ServiceInstanceId);

            if (ShouldDeployInterRelay(instance, action) == false)
            {
                await _publisher.PublishGatewayDeleteNetAppEntryAsync(thisLocation, instance.Name, actionPlanId,
                    instance.ServiceInstanceId);
            }

            _logger.LogDebug("Deleting relation between instance '{instanceName}'and location '{locationName}'",
                instance.Name, thisLocation.Name);

            await _redisInterfaceClient.DeleteRelationAsync(instance, thisLocation.ToBaseLocation(), "LOCATED_AT");
        }

        if (action.ShouldUseInterRelayForRosNetApps())
        {
            var labels = _kubeObjectBuilder.CreateInterRelayNetAppLabels(actionPlanId, actionId);
            var deleteRelayResult = await _kubernetesWrapper.TerminateInterRelayNetApp(labels);
            if (deleteRelayResult.IsSuccess)
            {
                await _publisher.PublishGatewayDeleteNetAppEntryAsync(thisLocation, deleteRelayResult.Value,
                    actionPlan.Id,
                    action.Id);
            }
        }
    }

    public async Task DeployActionAsync(Guid actionPlanId, Guid actionId)
    {
        _logger.LogTrace("Entered DeployActionAsync");
        _logger.LogDebug("Retrieving ActionPlan");
        var actionPlan = await _redisInterfaceClient.ActionPlanGetByIdAsync(actionPlanId);
        if (actionPlan is null)
            return;
        _logger.LogDebug("Retrieving action containing desired Service");
        var action = actionPlan.ActionSequence!.FirstOrDefault(a => a.Id == actionId);

        if (action is null)
            return;

        var systemCfg = await _systemConfigRepository.GetConfigAsync();
        var thisLocation = await GetCurrentLocationAsync();
        var deploymentNames = await _kubernetesWrapper.GetCurrentlyDeployedNetApps();
        var cfg = Config.Config.NewConfig
            .WithSystem(systemCfg)
            .WithLocation(thisLocation)
            .WithMiddleware(_mwConfig.Value)
            .WithDeploymentNames(deploymentNames);


        var deploymentPairs = await ConstructDeployments(action, cfg, true);

        foreach (var pair in deploymentPairs)
        {
            try
            {
                await _kubernetesWrapper.DeployNetApp(pair);
                // add relation between instance and location
                if (ShouldDeployInterRelay(pair.Instance, action) == false)
                {
                    await _publisher.PublishGatewayAddNetAppEntryAsync(thisLocation, pair.Name, actionPlan.Id,
                        pair.InstanceId);
                    var address = thisLocation.GetNetAppAddress(pair.Name);
                    
                    pair.Instance!.SetNetAppAddress(address);
                }

                _logger.LogDebug("Adding new relation between instance and current location");
                await _redisInterfaceClient.AddRelationAsync(pair.Instance!, thisLocation.ToBaseLocation(),
                    "LOCATED_AT");
                pair.Instance!.SetStatus(ServiceStatus.Active);
            }
            catch (HttpOperationException ex)
            {
                _logger.LogError(ex, "There was an error while deploying the service {service} caused by {reason}",
                    pair.Name, ex.Response.Content);
            }
        }

        if (action.ShouldUseInterRelayForRosNetApps())
        {
            var interRelay =
                _kubeObjectBuilder.CreateInterRelayNetAppDeploymentConfig(actionPlanId, action, deploymentPairs,
                    systemCfg);
            await _kubernetesWrapper.DeployNetApp(interRelay);

            foreach (var pair in deploymentPairs)
            {
                var address = thisLocation.GetNetAppAddress(interRelay.Name);
                
                pair.Instance!.SetNetAppAddress(address);
            }

            await _publisher.PublishGatewayAddNetAppEntryAsync(thisLocation, interRelay.Name, actionPlanId, action.Id);
        }

        _logger.LogDebug("Saving updated ActionPlan");
        await _redisInterfaceClient.ActionPlanAddAsync(actionPlan);
    }

    public async Task<Result> DeployActionPlanAsync(TaskModel task, Guid robotId)
    {
        var retVal = Result.Success();
        _logger.LogDebug("Entered DeploymentService.DeployAsync");
        
        var systemConfig = await _systemConfigRepository.GetConfigAsync();
        var location = await GetCurrentLocationAsync();

        var deploymentNames = await _kubernetesWrapper.GetCurrentlyDeployedNetApps();
        var config = Config.Config.NewConfig
            .WithSystem(systemConfig)
            .WithLocation(location)
            .WithMiddleware(_mwConfig.Value)
            .WithNetAppDataKey(task.NetAppDataKey)
            .WithDeploymentNames(deploymentNames);

        var robotResp = await _redisInterfaceClient.RobotGetByIdAsync(robotId);
        var robot = robotResp.ToRobot();

        try
        {
            var (deploymentQueue, relays)
                = await DeploymentQueue(task, config);

            retVal = await DeployQueue(task, deploymentQueue, location);

            await DeployRelays(task, relays, location);

            retVal += await SaveActionSequence(task, robot);
        }
        catch (NotInK8SEnvironmentException)
        {
            _logger.LogInformation("The instantiation of the kubernetes client has failed in {env} environment.",
                AppConfig.AppConfiguration);

            var isSuccess = AppConfig.AppConfiguration == AppVersionEnum.Dev.GetStringValue();

            if (isSuccess)
            {
                retVal += await SaveActionSequence(task, robot);
                _logger.LogWarning("Deployment of the services has been skipped in the Development environment");
            }

            return retVal;
        }
        catch (HttpOperationException ex)
        {
            _logger.LogError(ex, "There was an error while deploying the service caused by {reason}",
                ex.Response.Content);
            return ex.Response.Content;
        }
        catch (Exception ex)
        {
            _logger.LogError(ex, "The deployment of the Action Plan has failed!");
            return ex.Message;
        }

        return retVal;
    }

    private async Task DeployRelays(TaskModel task, Dictionary<Guid, DeploymentPair> relays, ILocation location)
    {
        foreach (var kvp in relays)
        {
            _logger.LogDebug("Deploying Inter Relay NetApp '{Name}' for action: {actionId}", kvp.Value.Name,
                kvp.Key);
            await _kubernetesWrapper.DeployNetApp(kvp.Value);
            await _publisher.PublishGatewayAddNetAppEntryAsync(location, kvp.Value.Name, task.ActionPlanId,
                kvp.Key);
        }
    }

    private async Task<Result> DeployQueue(TaskModel task,
        Dictionary<ActionModel, IReadOnlyList<DeploymentPair>> deploymentQueue, ILocation location)
    {
        var retVal = Result.Success();
        foreach (var item in deploymentQueue)
        {
            foreach (var pair in item.Value)
            {
                _logger.LogDebug("Deploying instance '{Name}', with serviceInstanceId '{ServiceInstanceId}'",
                    pair.Instance!.Name, pair.InstanceId);
                retVal += await _kubernetesWrapper.DeployNetApp(pair);

                if (ShouldDeployInterRelay(pair.Instance, item.Key) == false)
                {
                    await _publisher.PublishGatewayAddNetAppEntryAsync(location, pair.Name, task.ActionPlanId,
                        pair.InstanceId);
                    var netAppAddress = location.GetNetAppAddress(pair.Name);
                    _logger.LogDebug("NetApp address to be set: {address}", netAppAddress);
                    
                    pair.Instance!.SetNetAppAddress(netAppAddress);
                }

                _logger.LogDebug("Adding new relation between instance and current location");
                retVal += await _redisInterfaceClient.AddRelationAsync(pair.Instance!, location.ToBaseLocation(),
                    "LOCATED_AT");
                pair.Instance.SetStatus(ServiceStatus.Active);
            }
        }

        return retVal;
    }

    private async Task<(Dictionary<ActionModel, IReadOnlyList<DeploymentPair>> deploymentQueue,
            Dictionary<Guid, DeploymentPair> relays)>
        DeploymentQueue(TaskModel task, Config.Config config)
    {
        var deploymentQueue = new Dictionary<ActionModel, IReadOnlyList<DeploymentPair>>();
        var relays = new Dictionary<Guid, DeploymentPair>();
        foreach (var action in task.ActionSequence!)
        {
            var dplTmp = await ConstructDeployments(action, config);
            deploymentQueue.Add(action, dplTmp);

            if (action.ShouldUseInterRelayForRosNetApps() == false) continue;

            _logger.LogDebug("Current deployments: {deployments}", string.Join(", ", config.DeploymentNames));
            var relay = _kubeObjectBuilder.CreateInterRelayNetAppDeploymentConfig(task.ActionPlanId, action,
                dplTmp, config.System);
            relays[action.Id] = relay;
            foreach (var pair in dplTmp)
            {
                var netAppAddress = config.Location.GetNetAppAddress(relay.Name);
                _logger.LogDebug("{netAppName} NetApp address to be set: {address}", pair.Name, netAppAddress);
                pair.Instance!.SetNetAppAddress(netAppAddress);
            }
        }

        return (deploymentQueue, relays);
    }

    private async Task<bool> IsServiceUsedInOtherActionPlans(Guid actionPlanId, Guid serviceInstanceId)
    {
        var actionPlans = await _redisInterfaceClient.ActionPlanGetAllAsync();

        if (actionPlans is null) return true;

        // select all instances from all action plans except of current one
        var filtered = actionPlans.Where(a => a.Id != actionPlanId)
            .SelectMany(a => a.ActionSequence!.SelectMany(x => x.Services))
            .Where(i => i.ServiceInstanceId == serviceInstanceId).ToList();

        return filtered.Count > 0;
    }

    private static bool ShouldDeployInterRelay(InstanceModel instance, ActionModel action)
    {
        return string.IsNullOrWhiteSpace(instance.RosDistro) == false && action.SingleNetAppEntryPoint;
    }

    private async Task<IReadOnlyList<DeploymentPair>> ConstructDeployments(ActionModel action, Config.Config config, bool isSwitchover = false)
    {
        _logger.LogDebug("Entered DeploymentService.ConstructDeployments for action {action}", action.Name);
        var deployments = new List<DeploymentPair>();
        foreach (var service in action.Services)
        {
            try
            {
                // BB: service can be reused, to be decided by the resource planner
                if (service.ServiceInstanceId != Guid.Empty && isSwitchover == false)
                {
                    _logger.LogInformation("Skipping deployment of service {service} as it is already deployed",
                        service.Name);
                    continue;
                }
                    
                
                var pair = await PrepareDeploymentPair(service, config);
                deployments.Add(pair);
            }
            catch (Exception ex)
            {
                _logger.LogError(ex, "There was an error while preparing service deployment: {service}", service.Name);
            }
        }

        _kubeObjectBuilder.ConfigureCrossNetAppConnection(deployments);

        return deployments;
    }

    private async Task<DeploymentPair> PrepareDeploymentPair(InstanceModel instance, Config.Config config)
    {
        _logger.LogDebug("Querying for the images for service {Id}", instance.Id);
        var imagesResponse = await _redisInterfaceClient.ContainerImageGetForInstanceAsync(instance.Id);

        var images = imagesResponse.ToContainersList();
        if (images is null || images.Any() == false)
            throw new IncorrectDataException("Image is not defined for the Instance deployment");

        _logger.LogDebug("Retrieved service with Id: {Id}", instance.Id);

        instance.ContainerImage = images.First();

        _logger.LogDebug("Preparing the image {ImageName}", instance.Name);

        if (config.DeploymentNames.Contains(instance.Name)) instance.Name = instance.Name.AddRandomSuffix();

        config.DeploymentNames.Add(instance.Name);
        var pair = await ConfigureDeploymentObjects(instance, config);

        instance.SetStatus(ServiceStatus.Instantiating);
        instance.ServiceInstanceId = pair.InstanceId;

        return pair;
    }

    private async Task<ILocation> GetCurrentLocationAsync()
    {
        _logger.LogDebug("Retrieving location details (cloud or edge)");
        return (await _redisInterfaceClient.GetLocationByNameAsync(_mwConfig.Value.InstanceName)).ToLocation();
    }

    private async Task<ILocation> GetLocationAsync(LocationType type, string name)
    {
        _logger.LogDebug("Retrieving location details (cloud or edge) for type {type}, name: {name}", type.ToString(),
            name);
        return (await _redisInterfaceClient.GetLocationByNameAsync(_mwConfig.Value.InstanceName)).ToLocation();
    }

    /// <summary>
    ///     Saves the specified task to the redis as an action plan
    /// </summary>
    /// <param name="task"></param>
    /// <param name="robot"></param>
    /// <returns></returns>
    private async Task<Result> SaveActionSequence(TaskModel task, RobotModel robot)
    {
        var actionPlan = new ActionPlanModel(task.ActionPlanId, task.Id, task.Name, task.ActionSequence!, robot.Id);
        actionPlan.SetStatus("active");
        actionPlan.ActionSequence!.ForEach(a => a.Services.ForEach(s => s.SetStatus(ServiceStatus.Active)));

        var result = await _redisInterfaceClient.ActionPlanAddAsync(actionPlan);
        //await _redisInterfaceClient.AddRelationAsync(robot, actionPlan, "OWNS");
        if (result) _logger.LogInformation("Successfully saved action plan with Id {id}", actionPlan.Id);
        return result ? Result.Success() : Result.Failure("Could not save action plan");
    }

    private async Task<DeploymentPair> ConfigureDeploymentObjects(InstanceModel instance, Config.Config config)
    {
        var cim = instance.ContainerImage;
        var instanceName = instance.Name;
        var instanceId = Guid.NewGuid();

        var deployment =
            _kubeObjectBuilder.DeserializeAndConfigureDeployment(cim!.K8SDeployment, instanceId, instanceName,
                config.Location);

        if (instance.IsPersistent && string.IsNullOrWhiteSpace(config.NetAppDataKey) == false)
        {
            deployment = _kubeObjectBuilder.EnableDataPersistence(deployment, config.System, config.NetAppDataKey);
        }

        IRosConnectionBuilder builder = null;

        if (string.IsNullOrEmpty(instance.RosDistro) == false)
        {
            var distroEnum = RosDistroHelper.FromName(instance.RosDistro);
            builder = await _rosConnectionBuilderFactory.CreateConnectionBuilder(distroEnum);
        }

        if (builder is not null)
        {
            
            var rosSpec = new RosSpec(instance.RosTopicsSub,
                instance.RosTopicsPub,
                instance.Services,
                instance.Transforms,
                instance.Actions);

            deployment = builder.EnableRosCommunication(deployment, rosSpec);
        }

        deployment = _kubeObjectBuilder.AddLinkerdAnnotation(deployment);
        
        var service = string.IsNullOrWhiteSpace(cim.K8SService)
            ? _kubeObjectBuilder.CreateDefaultService(instanceName, instanceId, deployment)
            : _kubeObjectBuilder.DeserializeAndConfigureService(cim.K8SService, instanceName, instanceId);

        if (builder is not null) service = builder.EnableRelayNetAppCommunication(service);

        return new(deployment, service, instanceId, instance);
    }


    
}