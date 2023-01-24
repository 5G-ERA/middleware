using System.Net;
using AutoMapper;
using Middleware.Common;
using Middleware.Common.Enums;
using Middleware.Common.Responses;
using Middleware.Models.Domain;
using Middleware.ResourcePlanner.ApiReference;
using KeyValuePair = Middleware.Models.Domain.KeyValuePair;


namespace Middleware.ResourcePlanner;

public interface IResourcePlanner
{
    Task<TaskModel> Plan(TaskModel taskModel, RobotModel robot);
    Task<TaskModel> RePlan(TaskModel taskModel, TaskModel oldTaskmMdel, RobotModel robot, bool fullReplan);
}

public class ResourcePlanner : IResourcePlanner 
{
    private readonly IApiClientBuilder _apiClientBuilder;
    private readonly IMapper _mapper;
    private readonly IEnvironment _env;
    private readonly ILogger _logger;


    public string Slice5gType { get; private set; } //eMBB, URLL, mMTC, MIoT, V2X
    public bool StandAlone5GParam { get; private set; } // Standalone 5G or none-standalone.
    public int NumSlicesTestBed { get; private set; } //number of slices available in testbed.

    public ResourcePlanner(IApiClientBuilder apiClientBuilder, IMapper mapper, IEnvironment env,
        ILogger<ResourcePlanner> logger)

    {
        _apiClientBuilder = apiClientBuilder;
        _mapper = mapper;
        _env = env;
        _logger = logger;

    }

    private async Task NetworkPlan(RobotModel robot)
    {
        var redisApiClient = _apiClientBuilder.CreateRedisApiClient();
        List<RedisInterface.ActivePolicy> activePolicies = (await redisApiClient.PolicyGetActiveAsync()).ToList();

        foreach (RedisInterface.ActivePolicy policy in activePolicies)
        { 
                if (policy.PolicyName == "Use5G")
                {
                        //TODO: Query testbed for number of slices and types.

                        foreach (DialogueModel question in robot.Questions)
                        {
                            if (question.Name == "StandAlone5G or NoneStandAlone5G")
                            {
                                KeyValuePair answer = question.Answer.First();
                                bool StandAlone5GParam = (bool)answer.Value;
                            }
                        }

                        foreach (DialogueModel question in robot.Questions)
                        {
                            if (question.Name == "What type of 5G slice")
                            {
                                KeyValuePair answer = question.Answer.First();
                                string Slice5gType = (string)answer.Value; // there is an upper limit of eight network slices that be used by a device
                            }//Nest template
                        }

                //TODO: Attach robot to slice in Redis graph


            }
                if (policy.PolicyName == "Use4G")
                {

                }
                if (policy.PolicyName == "UseWifi")
                {

                }

        }

    }

    /// <summary>
    ///  Return placement based upon AllContainersInCloudMachine policy.
    /// </summary>
    /// <param name="replan"></param>
    /// <param name="robot"></param>
    /// <param name="actionParam"></param>
    /// <param name="resourceName"></param>
    /// <returns>string</returns>
    private async Task<string> ResourcesInCloud(bool replan, RobotModel robot, ActionModel actionParam, string resourceName)
    {
        var redisApiClient = _apiClientBuilder.CreateRedisApiClient();

        //Get free clouds
        List<RedisInterface.CloudModel> riconnectedClouds = (await redisApiClient.RobotGetConnectedCloudsIdsAsync(robot.Id)).ToList();
        List<RedisInterface.CloudModel> rifreeClouds = (await redisApiClient.GetFreeCloudIdsAsync(riconnectedClouds)).ToList();
        List<CloudModel> freeClouds = _mapper.Map<List<CloudModel>>(rifreeClouds);

        // If all of them are busy, check which one is less busy
        List<RedisInterface.CloudModel> rilessBusyClouds = (await redisApiClient.GetLessBusyCloudsAsync(riconnectedClouds)).ToList();
        List<CloudModel> lessBusyClouds = _mapper.Map<List<CloudModel>>(rilessBusyClouds);

        List<CloudModel> cloudsThatMeetNetAppRequirementsTotal = new List<CloudModel>();

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

                foreach(InstanceModel instance in actionParam.Services)
                {
                    // Check with BB
                     cloudsThatMeetNetAppRequirementsTotal.AddRange( freeClouds
                    .Where(cloud => cloud.NumberOfCores <= instance.MinimumNumCores &&
                            cloud.Ram <= actionParam.MinimumRam)
                    .ToList());
                }
                
                CloudModel freeCloudNodes = cloudsThatMeetNetAppRequirementsTotal.FirstOrDefault();
                if (freeCloudNodes is not null)
                    return freeCloudNodes.Name;
            }
        }

        // Replan flag is true
        else
        {
            // Get current HW resources of previosly selected edge
            RedisInterface.CloudModel riCloudData = await redisApiClient.CloudGetDataByNameAsync(resourceName);
            CloudModel cloudCurrentData = _mapper.Map<CloudModel>(riCloudData);

            if (freeClouds.Count() != 0)
            {
                //If there are free clouds, get a better cloud from the HW perspective for the netApp
                foreach (CloudModel freeCandidateCloud in freeClouds)
                {
                    if ((freeCandidateCloud.NumberOfCores > cloudCurrentData.NumberOfCores) && (freeCandidateCloud.Ram > cloudCurrentData.Ram))
                    {
                        return freeCandidateCloud.Name;
                    }
                }
            }

            else //There are no clouds free
            {
                // Get a better less busy cloud from the HW perspective for the netApp
                foreach (CloudModel lessBusyCandidatesClouds in lessBusyClouds)
                {
                    if ((lessBusyCandidatesClouds.NumberOfCores > cloudCurrentData.NumberOfCores) && (lessBusyCandidatesClouds.Ram > cloudCurrentData.Ram))
                    {
                        return lessBusyCandidatesClouds.Name;
                    }
                }
            }

        }
        return resourceName;
        //throw new InvalidOperationException("Coudnt not find a placement according to the active policies.");


    }

    /// <summary>
    ///  Return placement location to be the robot that requested a plan
    /// </summary>
    /// <param name="robot"></param>
    /// <param name="actionParam"></param>
    /// <returns>string</returns>
    /// <exception cref="Exception"></exception>
    private Task<string> ResourcesInRequestedTaskRobot(RobotModel robot, ActionModel actionParam)
    {
        //Check if the robot can handle the HW requirements of instance (NetApp's)
        foreach(InstanceModel instance in actionParam.Services)
        {
            if ((robot.NumberCores < actionParam.MinimumNumCores) && (robot.Ram < actionParam.MinimumRam))
            {
                //TODO: handle this other way.
                throw new Exception("The robot with ID " + robot.Id + "doesnt have the HW requirements to run the netApp with ID: " + actionParam.Id);
            }
        }
            // Select the placement to te the robot
            return Task.FromResult(robot.Name);//guid
    }

    /// <summary>
    ///  Return placement based upon AllContainersInClosestmachine policy
    /// </summary>
    /// <param name="replan"></param>
    /// <param name="robot"></param>
    /// <param name="actionParam"></param>
    /// <param name="resourceName"></param>
    /// <returns>string</returns>
    private async Task<string> ResourcesInClosestMachine(bool replan, RobotModel robot, ActionModel actionParam, string resourceName)
    {
        var redisApiClient = _apiClientBuilder.CreateRedisApiClient();

        // Get free edges
        List<RedisInterface.EdgeModel> riconnectedEdges = (await redisApiClient.RobotGetConnectedEdgesIdsAsync(robot.Id)).ToList();
        List<RedisInterface.EdgeModel> rifreeEdges = (await redisApiClient.GetFreeEdgesIdsAsync(riconnectedEdges)).ToList();
        List<EdgeModel> freeEdges = _mapper.Map<List<EdgeModel>>(rifreeEdges);

        // If all of them are busy, check which one is less busy
        List<RedisInterface.EdgeModel> rilessBusyEdges = (await redisApiClient.GetLessBusyEdgesAsync(riconnectedEdges)).ToList();
        List<EdgeModel> lessBusyEdges = _mapper.Map<List<EdgeModel>>(rilessBusyEdges);

        List<EdgeModel> edgesThatMeetNetAppRequirementsTotal = new List<EdgeModel>();

        // If replan flag is false
        if (replan == false)
        {
            // There are no free edges
            if (freeEdges.Count() == 0)
            {
                // Remove edges that do not have minimunm NetApps HW requirements
                var edgesThatMeetNetAppRequirements = lessBusyEdges
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
                foreach(InstanceModel instance in actionParam.Services)
                {
                    edgesThatMeetNetAppRequirementsTotal.AddRange(freeEdges
                    .Where(edge => edge.NumberOfCores <= actionParam.MinimumNumCores &&
                            edge.Ram <= actionParam.MinimumRam)
                    .ToList());
                }
                
                EdgeModel freeEdgesNodes = edgesThatMeetNetAppRequirementsTotal.FirstOrDefault();
                if (freeEdgesNodes is not null)
                    return freeEdgesNodes.Name;
            }


        }
        // Replan flag is true
        else 
        {
            // Get current HW resources of previosly selected edge
            RedisInterface.EdgeModel riEdgeData = await redisApiClient.EdgeGetDataByNameAsync(resourceName);
            EdgeModel edgeCurrentData = _mapper.Map<EdgeModel>(riEdgeData);

            if (freeEdges.Count() != 0)
            {
                //If there are free edges, get a better edge from the HW perspective for the netApp
                foreach (EdgeModel freeCandidateEdge in freeEdges)
                {
                    if ((freeCandidateEdge.NumberOfCores > edgeCurrentData.NumberOfCores) && (freeCandidateEdge.Ram > edgeCurrentData.Ram))
                    {
                        return freeCandidateEdge.Name;
                    }
                }
            }
            else //There are no edge free
            {
                // Get a better less busy edge from the HW perspective for the netApp
                foreach (EdgeModel lessBusyCandidatesEdges in lessBusyEdges)
                {
                    if ((lessBusyCandidatesEdges.NumberOfCores > edgeCurrentData.NumberOfCores) && (lessBusyCandidatesEdges.Ram > edgeCurrentData.Ram))
                    {
                        return lessBusyCandidatesEdges.Name;
                    }
                    
                }
            }

        }
        return resourceName;
        //throw new InvalidOperationException("Coudnt not find a placement according to the active policies.");

    }

    private async Task<string> InferResource (ActionModel actionParam, RobotModel robot, bool rePlan, List<ActionModel> candidates) //Allocate correct placement based upon policies and priority
    {
        bool ActionToConsider = false;
        // Check if this action requires infering a new placement
        foreach (ActionModel action in candidates)
        {
            if ((actionParam.Name == action.Name) && (rePlan==true))
            {
                ActionToConsider = true;
            }
        }

        // Check if this action requires infering a new placement
        if (rePlan == false)
        {
            ActionToConsider = true;
        }

        if (ActionToConsider == true)
        {
            var redisApiClient = _apiClientBuilder.CreateRedisApiClient();
            List<RedisInterface.ActivePolicy> activePolicies = (await redisApiClient.PolicyGetActiveAsync()).ToList(); //TODO: Only get policies of type resource.

            Dictionary<Guid, List<EdgeModel>> tempDic = new Dictionary<Guid, List<EdgeModel>>();//Diccionary of Key policy ID, values List of results after query
            string resourceName = actionParam.Placement;

            // Check all policies and decide placement for action.
            foreach (RedisInterface.ActivePolicy policy in activePolicies)
            {
                if (policy == null)
                {
                    throw new ArgumentException("Index policy is empty"); //This should never happend
                }
                //Store all in edge devices.
                if (policy.PolicyName == "AllContainersInClosestMachine")
                {
                    actionParam.Placement = await ResourcesInClosestMachine(rePlan, robot, actionParam, resourceName);
                }
                //Store all in the robot.
                if (policy.PolicyName == "runNetAppInRobot")
                {
                    actionParam.Placement = await ResourcesInRequestedTaskRobot(robot, actionParam);
                }
                //Store all in the cloud.
                if (policy.PolicyName == "AllContainersInCloud")
                {
                    actionParam.Placement = await ResourcesInCloud(rePlan, robot, actionParam, resourceName);     
                }
               
            }
            // Return to the old placement - TODO: check if the placement is not fully busy.
            return actionParam.Placement;
        }
        else
        {
            // Return to the old placement.
            return actionParam.Placement;
        }  
    }


    public async Task<TaskModel> Plan(TaskModel taskModel, RobotModel robot)
    {
        var list = new List<ActionModel>();
        // modify the existing plan with the candidates
        return await Plan(taskModel, robot, list); 
    }


    /// <summary>
    /// Plan from the resource level. Create relation for instancse and decide placement.
    /// </summary>
    /// <param name="taskModel"></param>
    /// <param name="robot"></param>
    /// <returns></returns>
    /// <exception cref="ArgumentException"></exception>
    protected async Task<TaskModel> Plan(TaskModel taskModel, RobotModel robot, List<ActionModel> actionCandidates)

    {
        var redisApiClient = _apiClientBuilder.CreateRedisApiClient();
        var orchestratorApiClient = _apiClientBuilder.CreateOrchestratorApiClient();
        // actionPlanner will give resource planner the actionSequence. 

        // Get action sequence 
        List<ActionModel> actionSequence = taskModel.ActionSequence;
        if (actionSequence == null || actionSequence.Count == 0)
            throw new ArgumentException("Action sequence cannot be empty");


        // iterate throught actions in actionSequence
        foreach (ActionModel action in actionSequence)
        {

            List<RedisInterface.RelationModel> imagesTmp =
                (await redisApiClient.ActionGetRelationByNameAsync(action.Id, "NEEDS")).ToList();
            List<RelationModel> images = new List<RelationModel>();
            foreach (RedisInterface.RelationModel imgTmp in imagesTmp)
            {
                RelationModel relation = _mapper.Map<RelationModel>(imgTmp);
                images.Add(relation);
            }

            // images -> list of all relations with images for the action
            foreach (RelationModel relation in images)
            {
                RedisInterface.InstanceModel instanceTmp = await redisApiClient.InstanceGetByIdAsync(relation.PointsTo.Id); //Get instance values

                InstanceModel instance = _mapper.Map<InstanceModel>(instanceTmp);

                if (CanBeReused(instance) && taskModel.ResourceLock == true)
                {
                    var reusedInstance = await GetInstanceToReuse(instance, orchestratorApiClient);
                    if (reusedInstance is not null)
                        instance = reusedInstance;
                }

                // add instance to actions
                action.Services.Add(instance);
            }
            // Choose placement based on policy
            action.Placement = await InferResource(action, robot,false, actionCandidates);
        }

        return taskModel;
    }

    /// <summary>
    /// Replan from the resource level. Create relation for instancse and decide placement.
    /// </summary>
    /// <param name="taskModel"></param>
    /// <param name="oldTaskmMdel"></param>
    /// <param name="robot"></param>
    /// <returns>Updated task sequence with new placements.</returns>
    /// <exception cref="ArgumentException"></exception>
    public async Task<TaskModel> RePlan(TaskModel taskModel, TaskModel oldTaskmMdel, RobotModel robot, bool fullReplan)
    {
        List<ActionModel> FailedActions = new List<ActionModel>();
        List<ActionModel> ActionsCandidates = new List<ActionModel>();

        // Get old action sequence 
        List<ActionModel> oldActionSequence = oldTaskmMdel.ActionSequence;
        if (oldActionSequence == null || oldActionSequence.Count == 0)
            throw new ArgumentException("Action sequence cannot be empty");

        // Get new action sequence 
        List<ActionModel> actionSequence = taskModel.ActionSequence;
        if (actionSequence == null || actionSequence.Count == 0)
            throw new ArgumentException("Action sequence cannot be empty");

        // Iterate throught old actions in actionSequence and check with have failed and add them to failed actions list.
        foreach (ActionModel oldAction in oldActionSequence)
        {
            if (oldAction.ActionStatus == "Failed")
            {
                FailedActions.Add(oldAction);
            }
        }

        // Check in which of the failed actions action planner has not done some modifications.
        foreach (ActionModel failedAction in FailedActions)
        {
            foreach(ActionModel newAction in actionSequence)
            {
                if (failedAction.Order == newAction.Order) //Compare old action with new one
                {
                    if (failedAction.Id == newAction.Id)
                    {
                        // Action planner did no change to the failed action.
                        ActionsCandidates.Add(failedAction);
                    }
                    
                }
            }
        }
        // If full replan was requested by robot.
        if (fullReplan == true)
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

    private async Task<InstanceModel> GetInstanceToReuse(InstanceModel instance, Orchestrator.OrchestratorApiClient orchestratorApi)

    {
        try
        {
            var statuses = (await orchestratorApi.NetAppStatusGetByInstanceIdAsync(instance.Id))?.ToList();
            if (statuses == null || statuses.Any() == false)
                return null;
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
            if (percentageUsage.Any() == false)
            {
                return null;
            }

            var instanceToReuse = percentageUsage.First();
            instance.ServiceInstanceId = instanceToReuse.Id;
            return instance;
        }
        catch (Orchestrator.ApiException<ApiResponse> apiEx)
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

    private bool CanBeReused(InstanceModel instance)
    {
        return instance.IsReusable != null && instance.IsReusable.Value;
    }

    
}