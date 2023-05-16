using System.Net;
using AutoMapper;
using Middleware.Common;
using Middleware.Common.Config;
using Middleware.Common.Enums;
using Middleware.Models.Domain;
using Middleware.RedisInterface.Contracts.Mappings;
using Middleware.RedisInterface.Contracts.Responses;
using Middleware.RedisInterface.Sdk;
using Middleware.ResourcePlanner.ApiReference;
using Middleware.ResourcePlanner.Models;
using Middleware.ResourcePlanner.Orchestrator;
using Middleware.ResourcePlanner.Policies;
using ActionModel = Middleware.Models.Domain.ActionModel;
using ApiResponse = Middleware.Common.Responses.ApiResponse;
using DialogueModel = Middleware.Models.Domain.DialogueModel;
using InstanceModel = Middleware.Models.Domain.InstanceModel;
using KeyValuePair = Middleware.Models.Domain.KeyValuePair;
using NetAppStatusModel = Middleware.ResourcePlanner.Orchestrator.NetAppStatusModel;
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
    private readonly IEnvironment _env;
    private readonly ILogger _logger;
    private readonly IMapper _mapper;
    private readonly MiddlewareConfig _mwConfig;
    private readonly IPolicyService _policyService;
    private readonly IRedisInterfaceClient _redisInterfaceClient;

    public ResourcePlanner(IApiClientBuilder apiClientBuilder, IMapper mapper, IEnvironment env,
        ILogger<ResourcePlanner> logger,
        IRedisInterfaceClient redisInterfaceClient,
        IConfiguration configuration,
        IPolicyService policyService)

    {
        _apiClientBuilder = apiClientBuilder;
        _mapper = mapper;
        _env = env;
        _logger = logger;
        _redisInterfaceClient = redisInterfaceClient;
        _policyService = policyService;
        _mwConfig = configuration.GetSection(MiddlewareConfig.ConfigName).Get<MiddlewareConfig>();
    }

    public async Task<TaskModel> Plan(TaskModel taskModel, RobotModel robot)
    {
        OrchestratorApiClient orchestratorApiClient = _apiClientBuilder.CreateOrchestratorApiClient();

        List<ActionModel> actionSequence = taskModel.ActionSequence;
        if (actionSequence == null || actionSequence.Count == 0)
            throw new ArgumentException("Action sequence cannot be empty");
        
        foreach (ActionModel action in actionSequence)
        {
            List<RelationModel> images = await _redisInterfaceClient.GetRelationAsync(action, "NEEDS");

            if (images == null) throw new Exception("The plan is not correctly configured");


            foreach (RelationModel relation in images)
            {
                InstanceResponse instanceResp = await _redisInterfaceClient.InstanceGetByIdAsync(relation.PointsTo.Id);
                InstanceModel instance = instanceResp.ToInstance();

                if (instance.CanBeReused() && taskModel.ResourceLock)
                {
                    InstanceModel reusedInstance = await GetInstanceToReuse(instance, orchestratorApiClient);
                    if (reusedInstance is not null)
                        instance = reusedInstance;
                }

                action.Services.Add(instance);
            }
            // find optimal location based on applied policies
            Location location = await _policyService.GetLocationAsync(action.Services);

            action.Placement = location.Name;
            action.PlacementType = location.Type;
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
    /// <returns>Updated task sequence with new placements.</returns>
    /// <exception cref="ArgumentException"></exception>
    public async Task<TaskModel> RePlan(TaskModel taskModel, TaskModel oldTask, RobotModel robot, bool isFullReplan)
    {
        var FailedActions = new List<ActionModel>();
        var ActionsCandidates = new List<ActionModel>();

        // Get old action sequence 
        List<ActionModel> oldActionSequence = oldTask.ActionSequence;
        if (oldActionSequence == null || oldActionSequence.Count == 0)
            throw new ArgumentException("Action sequence cannot be empty");

        // Get new action sequence 
        List<ActionModel> actionSequence = taskModel.ActionSequence;
        if (actionSequence == null || actionSequence.Count == 0)
            throw new ArgumentException("Action sequence cannot be empty");

        // Iterate throught old actions in actionSequence and check with have failed and add them to failed actions list.
        foreach (ActionModel oldAction in oldActionSequence)
            if (oldAction.ActionStatus == "Failed")
                FailedActions.Add(oldAction);

        // Check in which of the failed actions action planner has not done some modifications.
        foreach (ActionModel failedAction in FailedActions)
        foreach (ActionModel newAction in actionSequence)
            if (failedAction.Order == newAction.Order) //Compare old action with new one
                if (failedAction.Id == newAction.Id)
                    // Action planner did no change to the failed action.
                    ActionsCandidates.Add(failedAction);

        // If full replan was requested by robot.
        if (isFullReplan)
        {
            // Make a new plan but considering only the actionCandidates and leaving the same the other actions.
            taskModel = await Plan(taskModel, robot, ActionsCandidates);
            taskModel.FullReplan = true;
        }

        // If full replan is neccesary because all actions in action sequence have failed.
        if (FailedActions.Count == oldActionSequence.Count)
        {
            // Make a new plan but considering only the actionCandidates and leaving the same the other actions.
            taskModel = await Plan(taskModel, robot, ActionsCandidates);
            taskModel.FullReplan = true;
        }
        // If partial replan is requested by robot.
        else
        {
            // Make a new plan but considering only the actionCandidates and leaving the same the other actions.
            taskModel = await Plan(taskModel, robot, ActionsCandidates);
            taskModel.PartialRePlan = true;
        }

        return taskModel;
    }

    private async Task NetworkPlan(RobotModel robot)
    {
        GetPoliciesResponse activePoliciesResp = await _redisInterfaceClient.PolicyGetActiveAsync();
        List<PolicyModel> activePolicies = activePoliciesResp.ToPolicyList();

        foreach (PolicyModel policy in activePolicies)
        {
            if (policy.Name == "Use5G")
            {
                //TODO: Query testbed for number of slices and types.

                foreach (DialogueModel question in robot.Questions)
                    if (question.Name == "StandAlone5G or NoneStandAlone5G")
                    {
                        KeyValuePair answer = question.Answer.First();
                        var StandAlone5GParam = (bool)answer.Value;
                    }

                foreach (DialogueModel question in robot.Questions)
                    if (question.Name == "What type of 5G slice")
                    {
                        KeyValuePair answer = question.Answer.First();
                        var
                            slice5GType =
                                (string)answer
                                    .Value; // there is an upper limit of eight network slices that be used by a device
                    } //Nest template

                //TODO: Attach robot to slice in Redis graph
            }

            if (policy.Name == "Use4G")
            {
            }

            if (policy.Name == "UseWifi")
            {
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
        GetCloudsResponse riConnectedClouds =
            await _redisInterfaceClient.RobotGetConnectedCloudsAsync(robot.Id);

        List<CloudModel> availableClouds = riConnectedClouds.ToCloudList();

        List<CloudModel> rifreeClouds =
            (await _redisInterfaceClient.GetFreeCloudIdsAsync(availableClouds)).ToCloudList();
        var freeClouds = _mapper.Map<List<CloudModel>>(rifreeClouds);

        // If all of them are busy, check which one is less busy
        GetCloudsResponse riLessBusyClouds =
            await _redisInterfaceClient.GetLessBusyCloudsAsync(availableClouds);
        List<CloudModel> lessBusyClouds = riLessBusyClouds.ToCloudList();

        var cloudsThatMeetNetAppRequirementsTotal = new List<CloudModel>();

        // If replan flag is false
        if (replan == false)
        {
            // There are no free clouds
            if (freeClouds.Count() == 0)
            {
                // Remove edges that do not have minimunm NetApps HW requirements
                List<CloudModel> cloudsThatMeetNetAppRequirements = lessBusyClouds
                    .Where(cloud => cloud.NumberOfCores <= actionParam.MinimumNumCores &&
                                    cloud.Ram <= actionParam.MinimumRam)
                    .ToList();
                CloudModel lessBusyCloud = cloudsThatMeetNetAppRequirements.FirstOrDefault();
                if (lessBusyCloud is not null)
                    return lessBusyCloud.Name;

                // If list is empty, return empty string. 
                if (cloudsThatMeetNetAppRequirements.Count() == 0)
                    return resourceName;
                //throw new InvalidOperationException("Coudnt not find a placement according to the active policies.");
            }

            //There are free clouds

            else
            {
                // Remove edges that do not have minimunm instance (NetApps) HW requirements

                foreach (InstanceModel instance in actionParam.Services)
                    // Check with BB
                    cloudsThatMeetNetAppRequirementsTotal.AddRange(freeClouds
                        .Where(cloud => cloud.NumberOfCores <= instance.MinimumNumCores &&
                                        cloud.Ram <= actionParam.MinimumRam)
                        .ToList());

                CloudModel freeCloudNodes = cloudsThatMeetNetAppRequirementsTotal.FirstOrDefault();
                if (freeCloudNodes is not null)
                    return freeCloudNodes.Name;
            }
        }

        // Replan flag is true
        else
        {
            // Get current HW resources of previosly selected edge
            CloudResponse riCloudData = await _redisInterfaceClient.CloudGetByNameAsync(resourceName);
            CloudModel cloudCurrentData = riCloudData.ToCloud();

            if (freeClouds.Count() != 0)
            {
                //If there are free clouds, get a better cloud from the HW perspective for the netApp
                foreach (CloudModel freeCandidateCloud in freeClouds)
                    if (freeCandidateCloud.NumberOfCores > cloudCurrentData.NumberOfCores &&
                        freeCandidateCloud.Ram > cloudCurrentData.Ram)
                        return freeCandidateCloud.Name;
            }

            else //There are no clouds free
            {
                // Get a better less busy cloud from the HW perspective for the netApp
                foreach (CloudModel lessBusyCandidatesClouds in lessBusyClouds)
                    if (lessBusyCandidatesClouds.NumberOfCores > cloudCurrentData.NumberOfCores &&
                        lessBusyCandidatesClouds.Ram > cloudCurrentData.Ram)
                        return lessBusyCandidatesClouds.Name;
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
        foreach (InstanceModel instance in actionParam.Services!)
            if (robot.NumberCores < actionParam.MinimumNumCores && robot.Ram < actionParam.MinimumRam)
                //TODO: handle this other way.
                throw new Exception("The robot with ID " + robot.Id +
                                    "doesnt have the HW requirements to run the netApp with ID: " + actionParam.Id);

        // Select the placement to te the robot
        return Task.FromResult(robot.Name); //guid
    }

    /// <summary>
    ///     Return placement based upon AllContainersInClosestmachine policy
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
        GetEdgesResponse riConnectedEdges = await _redisInterfaceClient.RobotGetConnectedEdgesIdsAsync(robot.Id);

        List<EdgeModel> availableEdges = riConnectedEdges.ToEdgeList();
        GetEdgesResponse rifreeEdges = await _redisInterfaceClient.GetFreeEdgesIdsAsync(availableEdges);
        List<EdgeModel> freeEdges = rifreeEdges.ToEdgeList();

        // If all of them are busy, check which one is less busy
        GetEdgesResponse rilessBusyEdges = await _redisInterfaceClient.GetLessBusyEdgesAsync(availableEdges);
        List<EdgeModel> lessBusyEdges = rilessBusyEdges.ToEdgeList();

        var edgesThatMeetNetAppRequirementsTotal = new List<EdgeModel>();

        // If replan flag is false
        if (replan == false)
        {
            // There are no free edges
            if (freeEdges.Count() == 0)
            {
                // Remove edges that do not have minimunm NetApps HW requirements
                List<EdgeModel> edgesThatMeetNetAppRequirements = lessBusyEdges
                    .Where(edge => edge.NumberOfCores <= actionParam.MinimumNumCores &&
                                   edge.Ram <= actionParam.MinimumRam)
                    .ToList();
                EdgeModel lessBusyEdge = edgesThatMeetNetAppRequirements.FirstOrDefault();
                if (lessBusyEdge is not null)
                    return lessBusyEdge.Name;

                // If list is empty, return empty string. TODO - Check with BB.
                if (edgesThatMeetNetAppRequirements.Count() == 0)
                    return resourceName;
                //throw new InvalidOperationException("Coudnt not find a placement according to the active policies.");
            }

            //There are free edges
            else
            {
                // Remove edges that do not have minimunm instance (NetApps) HW requirements
                foreach (InstanceModel instance in actionParam.Services)
                    edgesThatMeetNetAppRequirementsTotal.AddRange(freeEdges
                        .Where(edge => edge.NumberOfCores <= actionParam.MinimumNumCores &&
                                       edge.Ram <= actionParam.MinimumRam)
                        .ToList());

                EdgeModel freeEdgesNodes = edgesThatMeetNetAppRequirementsTotal.FirstOrDefault();
                if (freeEdgesNodes is not null)
                    return freeEdgesNodes.Name;
            }
        }
        // Replan flag is true
        else
        {
            // Get current HW resources of previosly selected edge
            EdgeResponse riEdgeData = await _redisInterfaceClient.EdgeGetByNameAsync(resourceName);
            EdgeModel edgeCurrentData = riEdgeData.ToEdge();

            if (freeEdges.Count() != 0)
            {
                //If there are free edges, get a better edge from the HW perspective for the netApp
                foreach (EdgeModel freeCandidateEdge in freeEdges)
                    if (freeCandidateEdge.NumberOfCores > edgeCurrentData.NumberOfCores &&
                        freeCandidateEdge.Ram > edgeCurrentData.Ram)
                        return freeCandidateEdge.Name;
            }
            else //There are no edge free
            {
                // Get a better less busy edge from the HW perspective for the netApp
                foreach (EdgeModel lessBusyCandidatesEdges in lessBusyEdges)
                    if (lessBusyCandidatesEdges.NumberOfCores > edgeCurrentData.NumberOfCores &&
                        lessBusyCandidatesEdges.Ram > edgeCurrentData.Ram)
                        return lessBusyCandidatesEdges.Name;
            }
        }

        return resourceName;
        //throw new InvalidOperationException("Coudnt not find a placement according to the active policies.");
    }

    private async Task<string> InferResource(ActionModel actionParam, RobotModel robot, bool rePlan,
        List<ActionModel> candidates) //Allocate correct placement based upon policies and priority
    {
        var ActionToConsider = false;
        // Check if this action requires infering a new placement
        foreach (ActionModel action in candidates)
            if (actionParam.Name == action.Name && rePlan)
                ActionToConsider = true;

        // Check if this action requires infering a new placement
        if (rePlan == false) ActionToConsider = true;

        if (ActionToConsider)
        {
            GetPoliciesResponse riActivePolicies = await _redisInterfaceClient.PolicyGetActiveAsync();
            //TODO: Only get policies of type resource.
            List<PolicyModel> activePolicies = riActivePolicies.ToPolicyList();

            // Diccionary of Key policy ID, values List of results after query
            var tempDic = new Dictionary<Guid, List<EdgeModel>>();
            var resourceName = actionParam.Placement;

            // Check all policies and decide placement for action.
            foreach (PolicyModel policy in activePolicies)
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
        OrchestratorApiClient orchestratorApiClient = _apiClientBuilder.CreateOrchestratorApiClient();
        // actionPlanner will give resource planner the actionSequence. 

        // Get action sequence 
        List<ActionModel> actionSequence = taskModel.ActionSequence;
        if (actionSequence == null || actionSequence.Count == 0)
            throw new ArgumentException("Action sequence cannot be empty");


        // iterate throught actions in actionSequence
        foreach (ActionModel action in actionSequence)
        {
            List<RelationModel> imagesTmp =
                await _redisInterfaceClient.GetRelationForActionAsync(action.Id, "NEEDS");
            var images = new List<RelationModel>();
            foreach (RelationModel imgTmp in imagesTmp)
            {
                var relation = _mapper.Map<RelationModel>(imgTmp);
                images.Add(relation);
            }

            // images -> list of all relations with images for the action
            foreach (RelationModel relation in images)
            {
                InstanceResponse instanceResponse =
                    await _redisInterfaceClient.InstanceGetByIdAsync(relation.PointsTo.Id);
                InstanceModel instance = instanceResponse.ToInstance();

                if (instance.CanBeReused() && taskModel.ResourceLock)
                {
                    InstanceModel reusedInstance = await GetInstanceToReuse(instance, orchestratorApiClient);
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
            List<NetAppStatusModel> statuses =
                (await orchestratorApi.NetAppStatusGetByInstanceIdAsync(instance.Id))?.ToList();
            if (statuses == null || statuses.Any() == false)
                return null;
            if (statuses.Count == 1)
            {
                NetAppStatusModel status = statuses.First();
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
}