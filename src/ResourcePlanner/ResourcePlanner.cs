using System.Net;
using AutoMapper;
using Middleware.Common.Config;
using Middleware.Common.Enums;
using Middleware.Models.Domain;
using Middleware.RedisInterface.Contracts.Mappings;
using Middleware.RedisInterface.Sdk;
using Middleware.ResourcePlanner.ApiReference;
using Middleware.ResourcePlanner.Orchestrator;
using Middleware.ResourcePlanner.Policies;
using ActionModel = Middleware.Models.Domain.ActionModel;
using ApiResponse = Middleware.Common.Responses.ApiResponse;
using InstanceModel = Middleware.Models.Domain.InstanceModel;
using RelationModel = Middleware.Models.Domain.RelationModel;
using RobotModel = Middleware.Models.Domain.RobotModel;
using TaskModel = Middleware.Models.Domain.TaskModel;

namespace Middleware.ResourcePlanner;

public interface IResourcePlanner
{
    Task<TaskModel> Plan(TaskModel taskModel, RobotModel robot);
    Task<TaskModel> SemanticPlan(TaskModel taskModel, RobotModel robot);
    Task<TaskModel> RePlan(TaskModel taskModel, TaskModel oldTask, RobotModel robot, bool isFullReplan);
}

internal class ResourcePlanner : IResourcePlanner
{
    private readonly IApiClientBuilder _apiClientBuilder;
    private readonly ILogger _logger;
    private readonly IMapper _mapper;
    private readonly IPolicyService _policyService;
    private readonly IRedisInterfaceClient _redisInterfaceClient;

    public ResourcePlanner(IApiClientBuilder apiClientBuilder, IMapper mapper,
        ILogger<ResourcePlanner> logger,
        IRedisInterfaceClient redisInterfaceClient,
        IConfiguration configuration,
        IPolicyService policyService)

    {
        _apiClientBuilder = apiClientBuilder;
        _mapper = mapper;
        _logger = logger;
        _redisInterfaceClient = redisInterfaceClient;
        _policyService = policyService;
        configuration.GetSection(MiddlewareConfig.ConfigName).Get<MiddlewareConfig>();
    }

    public async Task<TaskModel> Plan(TaskModel taskModel, RobotModel robot)
    {
        var orchestratorApiClient = _apiClientBuilder.CreateOrchestratorApiClient();

        var actionSequence = taskModel.ActionSequence;
        if (actionSequence == null || actionSequence.Count == 0)
            throw new ArgumentException("Action sequence cannot be empty");

        foreach (var action in actionSequence)
        {
            var images = await _redisInterfaceClient.GetRelationAsync(action, "NEEDS");

            if (images == null) throw new("The plan is not correctly configured");


            foreach (var relation in images)
            {
                var instanceResp = await _redisInterfaceClient.InstanceGetByIdAsync(relation.PointsTo.Id);
                var instance = instanceResp.ToInstance();

                if (instance.CanBeReused() && taskModel.ResourceLock)
                {
                    var reusedInstance = await GetInstanceToReuse(instance, orchestratorApiClient);
                    if (reusedInstance is not null)
                        instance = reusedInstance;
                }

                action.Services.Add(instance);
            }

            // find optimal location based on applied policies
            var location = await _policyService.GetLocationAsync(action.Services);

            action.SetNewLocationForPlan(location);
        }

        return taskModel;
    }


    public async Task<TaskModel> SemanticPlan(TaskModel taskModel, RobotModel robot)
    {
        var list = new List<ActionModel>();
        // modify the existing plan with the candidates
        return await Plan(taskModel, robot, list);
    }

    /// <summary>
    ///     Replan from the resource level. Create relation for Instance and decide placement.
    /// </summary>
    /// <param name="taskModel"></param>
    /// <param name="oldTask"></param>
    /// <param name="robot"></param>
    /// <param name="isFullReplan"></param>
    /// <returns>Updated task sequence with new placements.</returns>
    /// <exception cref="ArgumentException"></exception>
    public async Task<TaskModel> RePlan(TaskModel taskModel, TaskModel oldTask, RobotModel robot, bool isFullReplan)
    {
        var failedActions = new List<ActionModel>();
        var actionsCandidates = new List<ActionModel>();

        // Get old action sequence 
        var oldActionSequence = oldTask.ActionSequence;
        if (oldActionSequence == null || oldActionSequence.Count == 0)
            throw new ArgumentException("Action sequence cannot be empty");

        // Get new action sequence 
        var actionSequence = taskModel.ActionSequence;
        if (actionSequence == null || actionSequence.Count == 0)
            throw new ArgumentException("Action sequence cannot be empty");

        // Iterate through old actions in actionSequence and check with have failed and add them to failed actions list.
        foreach (var oldAction in oldActionSequence)
        {
            if (oldAction.ActionStatus == "Failed")
                failedActions.Add(oldAction);
        }

        // Check in which of the failed actions action planner has not done some modifications.
        foreach (var failedAction in failedActions)
        foreach (var newAction in actionSequence)
        {
            if (failedAction.Order == newAction.Order) //Compare old action with new one
            {
                if (failedAction.Id == newAction.Id)
                    // Action planner did no change to the failed action.
                    actionsCandidates.Add(failedAction);
            }
        }

        // If full replan was requested by robot.
        if (isFullReplan)
        {
            // Make a new plan but considering only the actionCandidates and leaving the same the other actions.
            taskModel = await Plan(taskModel, robot, actionsCandidates);
            taskModel.FullReplan = true;
        }

        // If full replan is necessary because all actions in action sequence have failed.
        if (failedActions.Count == oldActionSequence.Count)
        {
            // Make a new plan but considering only the actionCandidates and leaving the same the other actions.
            taskModel = await Plan(taskModel, robot, actionsCandidates);
            taskModel.FullReplan = true;
        }
        // If partial replan is requested by robot.
        else
        {
            // Make a new plan but considering only the actionCandidates and leaving the same the other actions.
            taskModel = await Plan(taskModel, robot, actionsCandidates);
            taskModel.PartialRePlan = true;
        }

        return taskModel;
    }

    private async Task NetworkPlan(RobotModel robot)
    {
        var activePoliciesResp = await _redisInterfaceClient.PolicyGetActiveAsync();
        var activePolicies = activePoliciesResp.ToPolicyList();

        foreach (var policy in activePolicies)
        {
            switch (policy.Name)
            {
                case "Use5G":
                {
                    //TODO: Query testbed for number of slices and types.
                    foreach (var question in robot.Questions!)
                    {
                        if (question.Name == "StandAlone5G or NoneStandAlone5G")
                        {
                            var answer = question.Answer!.First();
                            var standAlone5GParam = (bool)answer.Value;
                        }
                    }

                    foreach (var question in robot.Questions)
                    {
                        if (question.Name == "What type of 5G slice")
                        {
                            var answer = question.Answer!.First();
                            // there is an upper limit of eight network slices that be used by a device
                            var slice5GType = (string)answer.Value;
                        } //Nest template
                    }

                    //TODO: Attach robot to slice in Redis graph
                    break;
                }
                case "Use4G":
                    break;
                case "UseWifi":
                    break;
            }
        }
    }

    /// <summary>
    ///     Return placement based upon AllContainersInCloudMachine policy.
    /// </summary>
    /// <param name="replan"></param>
    /// <param name="robot"></param>
    /// <param name="actionParam"></param>
    /// <param name="resourceName"></param>
    /// <returns>string</returns>
    private async Task<string> ResourcesInCloud(bool replan, RobotModel robot, ActionModel actionParam,
        string resourceName)
    {
        //Get free clouds
        var riConnectedClouds =
            await _redisInterfaceClient.RobotGetConnectedCloudsAsync(robot.Id);

        var availableClouds = riConnectedClouds.ToCloudList();

        var rifreeClouds =
            (await _redisInterfaceClient.GetFreeCloudIdsAsync(availableClouds)).ToCloudList();
        var freeClouds = _mapper.Map<List<CloudModel>>(rifreeClouds);

        // If all of them are busy, check which one is less busy
        var riLessBusyClouds =
            await _redisInterfaceClient.GetLessBusyCloudsAsync(availableClouds);
        var lessBusyClouds = riLessBusyClouds.ToCloudList();

        var cloudsThatMeetNetAppRequirementsTotal = new List<CloudModel>();

        // If replan flag is false
        if (replan == false)
        {
            // There are no free clouds
            if (freeClouds.Count() == 0)
            {
                // Remove edges that do not have minimunm NetApps HW requirements
                var cloudsThatMeetNetAppRequirements = lessBusyClouds
                    .Where(cloud => cloud.NumberOfCores <= actionParam.MinimumNumCores &&
                                    cloud.Ram <= actionParam.MinimumRam)
                    .ToList();
                var lessBusyCloud = cloudsThatMeetNetAppRequirements.FirstOrDefault();
                if (lessBusyCloud is not null)
                    return lessBusyCloud.Name;

                // If list is empty, return empty string. 
                if (cloudsThatMeetNetAppRequirements.Count == 0)
                    return resourceName;
                //throw new InvalidOperationException("Coudnt not find a placement according to the active policies.");
            }

            //There are free clouds

            else
            {
                // Remove edges that do not have minimunm instance (NetApps) HW requirements

                foreach (var instance in actionParam.Services)
                    // Check with BB
                {
                    cloudsThatMeetNetAppRequirementsTotal.AddRange(freeClouds
                        .Where(cloud => cloud.NumberOfCores <= instance.MinimumNumCores &&
                                        cloud.Ram <= actionParam.MinimumRam)
                        .ToList());
                }

                var freeCloudNodes = cloudsThatMeetNetAppRequirementsTotal.FirstOrDefault();
                if (freeCloudNodes is not null)
                    return freeCloudNodes.Name;
            }
        }

        // Replan flag is true
        else
        {
            // Get current HW resources of previosly selected edge
            var riCloudData = await _redisInterfaceClient.CloudGetByNameAsync(resourceName);
            var cloudCurrentData = riCloudData.ToCloud();

            if (freeClouds.Count() != 0)
            {
                //If there are free clouds, get a better cloud from the HW perspective for the netApp
                foreach (var freeCandidateCloud in freeClouds)
                {
                    if (freeCandidateCloud.NumberOfCores > cloudCurrentData.NumberOfCores &&
                        freeCandidateCloud.Ram > cloudCurrentData.Ram)
                        return freeCandidateCloud.Name;
                }
            }

            else //There are no clouds free
            {
                // Get a better less busy cloud from the HW perspective for the netApp
                foreach (var lessBusyCandidatesClouds in lessBusyClouds)
                {
                    if (lessBusyCandidatesClouds.NumberOfCores > cloudCurrentData.NumberOfCores &&
                        lessBusyCandidatesClouds.Ram > cloudCurrentData.Ram)
                        return lessBusyCandidatesClouds.Name;
                }
            }
        }

        return resourceName;
        //throw new InvalidOperationException("Coudnt not find a placement according to the active policies.");
    }

    /// <summary>
    ///     Return placement location to be the robot that requested a plan
    /// </summary>
    /// <param name="robot"></param>
    /// <param name="actionParam"></param>
    /// <returns>string</returns>
    /// <exception cref="Exception"></exception>
    private Task<string> ResourcesInRequestedTaskRobot(RobotModel robot, ActionModel actionParam)
    {
        //Check if the robot can handle the HW requirements of instance (NetApp's)
        foreach (var instance in actionParam.Services!)
        {
            if (robot.NumberCores < actionParam.MinimumNumCores && robot.Ram < actionParam.MinimumRam)
                //TODO: handle this other way.
            {
                throw new("The robot with ID " + robot.Id +
                          "doesn't have the HW requirements to run the netApp with ID: " + actionParam.Id);
            }
        }

        // Select the placement to te the robot
        return Task.FromResult(robot.Name); //guid
    }

    /// <summary>
    ///     Return placement based upon AllContainersInClosestMachine policy
    /// </summary>
    /// <param name="replan"></param>
    /// <param name="robot"></param>
    /// <param name="actionParam"></param>
    /// <param name="resourceName"></param>
    /// <returns>string</returns>
    private async Task<string> ResourcesInClosestMachine(bool replan, RobotModel robot, ActionModel actionParam,
        string resourceName)
    {
        // Get free edges
        var riConnectedEdges = await _redisInterfaceClient.RobotGetConnectedEdgesIdsAsync(robot.Id);

        var availableEdges = riConnectedEdges.ToEdgeList();
        var riFreeEdges = await _redisInterfaceClient.GetFreeEdgesIdsAsync(availableEdges);
        var freeEdges = riFreeEdges.ToEdgeList();

        // If all of them are busy, check which one is less busy
        var riLessBusyEdges = await _redisInterfaceClient.GetLessBusyEdgesAsync(availableEdges);
        var lessBusyEdges = riLessBusyEdges.ToEdgeList();

        var edgesThatMeetNetAppRequirementsTotal = new List<EdgeModel>();

        // If replan flag is false
        if (replan == false)
        {
            // There are no free edges
            if (!freeEdges.Any())
            {
                // Remove edges that do not have minimunm NetApps HW requirements
                var edgesThatMeetNetAppRequirements = lessBusyEdges
                    .Where(edge => edge.NumberOfCores <= actionParam.MinimumNumCores &&
                                   edge.Ram <= actionParam.MinimumRam)
                    .ToList();
                var lessBusyEdge = edgesThatMeetNetAppRequirements.FirstOrDefault();
                if (lessBusyEdge is not null)
                    return lessBusyEdge.Name;

                // If list is empty, return empty string. TODO - Check with BB.
                if (!edgesThatMeetNetAppRequirements.Any())
                    return resourceName;
                //throw new InvalidOperationException("Coudnt not find a placement according to the active policies.");
            }

            //There are free edges
            else
            {
                // Remove edges that do not have minimum instance (NetApps) HW requirements
                foreach (var instance in actionParam.Services)
                {
                    edgesThatMeetNetAppRequirementsTotal.AddRange(freeEdges
                        .Where(edge => edge.NumberOfCores <= actionParam.MinimumNumCores &&
                                       edge.Ram <= actionParam.MinimumRam)
                        .ToList());
                }

                var freeEdgesNodes = edgesThatMeetNetAppRequirementsTotal.FirstOrDefault();
                if (freeEdgesNodes is not null)
                    return freeEdgesNodes.Name;
            }
        }
        // Replan flag is true
        else
        {
            // Get current HW resources of previously selected edge
            var riEdgeData = await _redisInterfaceClient.EdgeGetByNameAsync(resourceName);
            var edgeCurrentData = riEdgeData.ToEdge();

            if (freeEdges.Count != 0)
            {
                //If there are free edges, get a better edge from the HW perspective for the netApp
                foreach (var freeCandidateEdge in freeEdges)
                {
                    if (freeCandidateEdge.NumberOfCores > edgeCurrentData.NumberOfCores &&
                        freeCandidateEdge.Ram > edgeCurrentData.Ram)
                        return freeCandidateEdge.Name;
                }
            }
            else //There are no edge free
            {
                // Get a better less busy edge from the HW perspective for the netApp
                foreach (var lessBusyCandidatesEdges in lessBusyEdges)
                {
                    if (lessBusyCandidatesEdges.NumberOfCores > edgeCurrentData.NumberOfCores &&
                        lessBusyCandidatesEdges.Ram > edgeCurrentData.Ram)
                        return lessBusyCandidatesEdges.Name;
                }
            }
        }

        return resourceName;
        //throw new InvalidOperationException("Couldn't not find a placement according to the active policies.");
    }

    private async Task<string> InferResource(ActionModel actionParam, RobotModel robot, bool rePlan,
        List<ActionModel> candidates) //Allocate correct placement based upon policies and priority
    {
        var actionToConsider = false;
        // Check if this action requires inferring a new placement
        foreach (var action in candidates)
        {
            if (actionParam.Name == action.Name && rePlan)
                actionToConsider = true;
        }

        // Check if this action requires inferring a new placement
        if (rePlan == false) actionToConsider = true;

        if (actionToConsider)
        {
            var riActivePolicies = await _redisInterfaceClient.PolicyGetActiveAsync();
            //TODO: Only get policies of type resource.
            var activePolicies = riActivePolicies.ToPolicyList();

            // Dictionary of Key policy ID, values List of results after query
            var tempDic = new Dictionary<Guid, List<EdgeModel>>();
            var resourceName = actionParam.Placement;

            // Check all policies and decide placement for action.
            foreach (var policy in activePolicies)
            {
                if (policy == null) throw new ArgumentException("Index policy is empty"); //This should never happend

                //Store all in edge devices.
                if (policy.Name == "AllContainersInClosestMachine")
                    actionParam.Placement = await ResourcesInClosestMachine(rePlan, robot, actionParam, resourceName);

                //Store all in the robot.
                if (policy.Name == "runNetAppInRobot")
                    actionParam.Placement = await ResourcesInRequestedTaskRobot(robot, actionParam);

                //Store all in the cloud.
                if (policy.Name == "AllContainersInCloud")
                    actionParam.Placement = await ResourcesInCloud(rePlan, robot, actionParam, resourceName);
            }

            // Return to the old placement - TODO: check if the placement is not fully busy.
            return actionParam.Placement;
        }

        // Return to the old placement.
        return actionParam.Placement;
    }


    /// <summary>
    ///     SemanticPlan from the resource level. Create relation for Instance and decide placement.
    /// </summary>
    /// <param name="taskModel"></param>
    /// <param name="robot"></param>
    /// <param name="actionCandidates"></param>
    /// <returns></returns>
    /// <exception cref="ArgumentException"></exception>
    protected async Task<TaskModel> Plan(TaskModel taskModel, RobotModel robot, List<ActionModel> actionCandidates)
    {
        var orchestratorApiClient = _apiClientBuilder.CreateOrchestratorApiClient();
        // actionPlanner will give resource planner the actionSequence. 

        // Get action sequence 
        var actionSequence = taskModel.ActionSequence;
        if (actionSequence == null || actionSequence.Count == 0)
            throw new ArgumentException("Action sequence cannot be empty");


        // iterate through actions in actionSequence
        foreach (var action in actionSequence)
        {
            var imagesTmp =
                await _redisInterfaceClient.GetRelationForActionAsync(action.Id, "NEEDS");
            var images = new List<RelationModel>();
            if (imagesTmp is null)
                continue;

            foreach (var imgTmp in imagesTmp)
            {
                var relation = _mapper.Map<RelationModel>(imgTmp);
                images.Add(relation);
            }

            // images -> list of all relations with images for the action
            foreach (var relation in images)
            {
                var instanceResponse =
                    await _redisInterfaceClient.InstanceGetByIdAsync(relation.PointsTo.Id);
                var instance = instanceResponse.ToInstance();

                if (instance.CanBeReused() && taskModel.ResourceLock)
                {
                    var reusedInstance = await GetInstanceToReuse(instance, orchestratorApiClient);
                    if (reusedInstance is not null)
                        instance = reusedInstance;
                }

                // add instance to actions
                action.Services!.Add(instance);
            }

            // Choose placement based on policy
            action.Placement = await InferResource(action, robot, false, actionCandidates);
        }

        return taskModel;
    }

    private async Task<InstanceModel> GetInstanceToReuse(InstanceModel instance,
        OrchestratorApiClient orchestratorApi)

    {
        try
        {
            var statuses =
                (await orchestratorApi.NetAppStatusGetByInstanceIdAsync(instance.Id))?.ToList();
            if (statuses == null || statuses.Any() == false)
            {
                return
                    await GetAlreadyDeployedNetAppAsync(
                        instance); // get already deployed instance when no heartbeat found
            }

            if (statuses.Count == 1)
            {
                var status = statuses.First();
                instance.ServiceInstanceId = status.Id;
                return instance;
            }

            var percentageUsage = statuses.Select(s =>
                    new
                    {
                        s.Id,
                        Percentage = s.CurrentRobotsCount / (decimal)s.HardLimit * 100,
                        // TODO: this will have to be expanded based on the resource prediction
                        // for example if the edge can accommodate another instance
                        Recommendation = s.CurrentRobotsCount < s.OptimalLimit ? NetAppStatus.Green :
                            s.CurrentRobotsCount >= s.HardLimit ? NetAppStatus.Red : NetAppStatus.Yellow
                    }).Where(s => s.Recommendation != NetAppStatus.Red)
                .OrderBy(s => s.Recommendation).ThenBy(s => s.Percentage).ToList();

            // need for the deployment of the new service no to overload one in the red zone
            if (percentageUsage.Any() == false) return null;

            var instanceToReuse = percentageUsage.First();
            instance.ServiceInstanceId = instanceToReuse.Id;
            return instance;
        }
        catch (ApiException<ApiResponse> apiEx)
        {
            if (apiEx.StatusCode == (int)HttpStatusCode.NotFound)
            {
                _logger.LogDebug("No active instances for {name} with {id} were found", instance.Name, instance.Id);
                return null;
            }

            _logger.LogError(apiEx, "There was a problem while calling the Orchestrator");
        }
        catch (Exception ex)
        {
            _logger.LogError(ex, "Unable to retrieve the status of the specified NetApps from the orchestrator");
        }

        return null;
    }

    private async Task<InstanceModel> GetAlreadyDeployedNetAppAsync(InstanceModel instance)
    {
        var allActionPlans = await _redisInterfaceClient.ActionPlanGetAllAsync();
        if (allActionPlans is null)
            return null;

        var instances = allActionPlans
            .SelectMany(a => a.ActionSequence!.SelectMany(x => x.Services))
            .Where(i => i.Id == instance.Id).ToList();

        if (instances.Any() == false) return null;
        var selected = instances.First();
        selected.ContainerImage = null;
        return selected;
    }
}