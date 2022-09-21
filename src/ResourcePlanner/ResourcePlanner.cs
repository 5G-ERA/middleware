using System.Linq;
using System.Net;
using AutoMapper;
using Middleware.Common;
using Middleware.Common.Enums;
using Middleware.Common.Models;
using Middleware.Common.Repositories;
using Middleware.ResourcePlanner.ApiReference;


namespace Middleware.ResourcePlanner;

public interface IResourcePlanner
{
    Task<TaskModel> Plan(TaskModel taskModel, RobotModel robot);
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

    public ResourcePlanner(IApiClientBuilder apiClientBuilder, IMapper mapper, IEnvironment env, ILogger<ResourcePlanner> logger)
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
                                Common.Models.KeyValuePair answer = question.Answer.First();
                                bool StandAlone5GParam = (bool)answer.Value;
                            }
                        }

                        foreach (DialogueModel question in robot.Questions)
                        {
                            if (question.Name == "What type of 5G slice")
                            {
                                Common.Models.KeyValuePair answer = question.Answer.First();
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

    private async Task<ActionModel> InferResource (ActionModel actionParam, RobotModel robot, bool rePlan) //Allocate correct placement based upon policies and priority
    {
        var redisApiClient = _apiClientBuilder.CreateRedisApiClient();
        List<RedisInterface.ActivePolicy> activePolicies = (await redisApiClient.PolicyGetActiveAsync()).ToList();
        Dictionary<Guid, List<EdgeModel>> tempDic = new Dictionary<Guid, List<EdgeModel>>();//Diccionary of Key policy ID, values List of results after query
        string resourceName = actionParam.Placement;
        
        if (rePlan == true)
        {
            // Get resource stadistics of all action locations.
            
            RedisInterface.CloudModel riCloudData = await redisApiClient.CloudGetDataByNameAsync(resourceName);
            CloudModel cloudCurrentData = _mapper.Map<CloudModel>(riCloudData);


 
        }

        foreach (RedisInterface.ActivePolicy policy in activePolicies)
        {
            if (policy == null)
            {
                throw new ArgumentException("Index policy is empty"); //This should never happend
            }
            if (policy.PolicyName == "AllContainersInClosestMachine") 
            {
                // Get free edges
                List<RedisInterface.EdgeModel> riconnectedEdges = (await redisApiClient.RobotGetConnectedEdgesIdsAsync(robot.Id)).ToList();
                List<RedisInterface.EdgeModel> rifreeEdges = (await redisApiClient.GetFreeEdgesIdsAsync(riconnectedEdges)).ToList();
                List<EdgeModel> freeEdges = _mapper.Map<List<EdgeModel>>(rifreeEdges);

                //If all of them are busy, check which one is less busy
                List<RedisInterface.EdgeModel> rilessBusyEdges = (await redisApiClient.GetLessBusyEdgesAsync(riconnectedEdges)).ToList();
                List<EdgeModel> lessBusyEdges = _mapper.Map<List<EdgeModel>>(rilessBusyEdges);

                if (rePlan == false)
                {       
                    if (freeEdges.Count()==0)
                    {
                        //Remove edges that do not have minimunm NetApps HW requirements
                        foreach (EdgeModel edgeCandidate in lessBusyEdges)
                        {
                            if ((edgeCandidate.NumberOfCores < actionParam.MinimumNumCores) && (edgeCandidate.Ram < actionParam.MinimumRam))
                            {
                                lessBusyEdges.Remove(edgeCandidate);
                            }
                        }
                        //tempDic.Add(policy.Id, lessBusyEdges);
                        EdgeModel LessBusyEdge = lessBusyEdges.First();
                        actionParam.Placement = LessBusyEdge.Name;
                        //Query to get minimunm resources of NetApp. actionParam.Id
                        }

                    //There are free edges
                    else
                    {
                        ////Remove edges that do not have minimunm NetApps HW requirements
                        foreach (EdgeModel edgeCandidate in freeEdges)
                        {
                            if ((edgeCandidate.NumberOfCores < actionParam.MinimumNumCores) && (edgeCandidate.Ram < actionParam.MinimumRam))
                            {
                                freeEdges.Remove(edgeCandidate);
                            }
                        }
                        //tempDic.Add(policy.Id, freeEdges);
                        EdgeModel freeEdgesObj = freeEdges.First();
                        actionParam.Placement = freeEdgesObj.Name;
                    }
                 
                }
                else // Replan is true
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
                                actionParam.Placement = freeCandidateEdge.Name;
                            }
                        }
                     }
                    else //There are no edge free
                    {
                        // Get a better edge from the HW perspective for the netApp
                        foreach (EdgeModel lessBusyCandidatesEdges in lessBusyEdges)
                        {
                            if ((lessBusyCandidatesEdges.NumberOfCores > edgeCurrentData.NumberOfCores) && (lessBusyCandidatesEdges.Ram > edgeCurrentData.Ram))
                            {
                                actionParam.Placement = lessBusyCandidatesEdges.Name;
                            }
                        }
                    }
                    
                }
            }
            //All stored in the robot.
            if (policy.PolicyName == "StoreAllInRobot") // Put all neccesary containers inside robot
            {
                //Check if the robot can handle the HW requirements of NetApp
                if ((robot.NumberCores < actionParam.MinimumNumCores) && (robot.Ram < actionParam.MinimumRam))
                {
                    throw new Exception("The robot with ID "+robot.Id+"doesnt have the HW requirements to run the netApp with ID: "+actionParam.Id);
                }
                else
                {
                    // Select the placement to te the robot
                    actionParam.Placement = robot.Name;//guid
                }
                
            }
                //QoS & QoE

        }

        return actionParam;
    }



    public async Task<TaskModel> Plan(TaskModel taskModel, RobotModel robot)
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
            List<RedisInterface.RelationModel> imagesTmp = (await redisApiClient.ActionGetRelationByNameAsync(action.Id, "NEEDS")).ToList();
            List<RelationModel> images = new List<RelationModel>();
            foreach (RedisInterface.RelationModel imgTmp in imagesTmp)
            {
                RelationModel relation = _mapper.Map<RelationModel>(imgTmp);
                images.Add(relation);
            }

            // images -> list of all relations with images for the action
            foreach (RelationModel relation in images)
            {
                RedisInterface.InstanceModel instanceTmp = await redisApiClient.InstanceGetByIdAsync(relation.PointsTo.Id);
                //map
                InstanceModel instance = _mapper.Map<InstanceModel>(instanceTmp);

                if (CanBeReused(instance) && taskModel.ResourceLock==true)
                {
                    var reusedInstance = await GetInstanceToReuse(instance, orchestratorApiClient);
                    if (reusedInstance is not null)
                        instance = reusedInstance;
                }
                // add instance to actions 
                action.Services.Add(instance);
            }
        }
        return taskModel;

    }

   
    public async Task<TaskModel> RePlan(TaskModel taskModel, TaskModel oldTaskmMdel, RobotModel robot)
    {
        

        List<ActionModel> FailedActions = new List<ActionModel>();
        List<ActionModel> ActionsModifiedCandidates = new List<ActionModel>();

        // Get old action sequence 
        List<ActionModel> oldActionSequence = oldTaskmMdel.ActionSequence;
        if (oldActionSequence == null || oldActionSequence.Count == 0)
            throw new ArgumentException("Action sequence cannot be empty");

        // Get new action sequence 
        List<ActionModel> actionSequence = taskModel.ActionSequence;
        if (actionSequence == null || actionSequence.Count == 0)
            throw new ArgumentException("Action sequence cannot be empty");

        // iterate throught old actions in actionSequence and complete failed actions list.
        foreach (ActionModel oldAction in oldActionSequence)
        {
            

            if (oldAction.ActionStatus == "Failed")
            {
                FailedActions.Add(oldAction);
            }
        }

        // Check in which of the failed actions action planner has done some modifications.
        foreach (ActionModel failedAction in FailedActions)
        {
            foreach(ActionModel newAction in actionSequence)
            {
                if (failedAction.Order == newAction.Order) //Compare old action with new one
                {
                    if (failedAction.Id == newAction.Id)
                    {
                        // Action planner did no change to the failed action.
                        ActionsModifiedCandidates.Add(failedAction);
                    }
                    
                }
            }
        }
        // If Full replan
        // TODO


        // If partial replan is requested:
        foreach (ActionModel actionFail in ActionsModifiedCandidates)
        {
            //ActionModel actionModel = InferResource();
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
                        Percentage = s.CurrentRobotsCount / (decimal) s.HardLimit * 100,
                        // TODO: this will have to be expanded based on the resource prediction
                        // for example if the edge can accommodate another instance
                        Recommendation = s.CurrentRobotsCount < s.OptimalLimit ? NetAppStatus.Green :
                            s.CurrentRobotsCount >= s.HardLimit ? NetAppStatus.Red : NetAppStatus.Yellow
                    }).Where(s => s.Recommendation != NetAppStatus.Red)
                .OrderBy(s => s.Recommendation).ThenBy(s=>s.Percentage).ToList();

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