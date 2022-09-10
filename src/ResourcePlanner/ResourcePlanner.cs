using System.Linq;
using System.Net;
using AutoMapper;
using Middleware.Common;
using Middleware.Common.Enums;
using Middleware.Common.Models;
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
        List<Middleware.ResourcePlanner.RedisInterface.ActivePolicy> activePolicies = (await redisApiClient.PolicyGetActiveAsync()).ToList();
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

    private async Task<ActionModel> InferResource (ActionModel actionParam, RobotModel robot) //Allocate correct placement based upon policies and priority
    {
        var redisApiClient = _apiClientBuilder.CreateRedisApiClient();
        List<Middleware.ResourcePlanner.RedisInterface.ActivePolicy> activePolicies = (await redisApiClient.PolicyGetActiveAsync()).ToList();
        Dictionary<Guid, List<String>> tempDic = new Dictionary<Guid, List<String>>();//Diccionary of Key policy ID, values List of results after query

        foreach (RedisInterface.ActivePolicy policy in activePolicies)
        {
            if (policy == null)
            {
                throw new ArgumentException("Index policy is empty"); //This should never happend
            }
            if (policy.PolicyName == "AllContainersInClosestMachine") // Resource allocation policies RobotGetConnectedEdgesIdsAsync(robot.Id)).ToList();
            {

                //List<EdgeModel> connectedEdges = (await redisApiClient.RobotGetConnectedEdgesIdsAsync(robot.Id)).ToList();


               // List<EdgeModel> freeEdges = (await redisApiClient.GetFreeEdgesIdsAsync(connectedEdges)).ToList();


                //if (freeEdges.Count()==0)
                //{
                //    //if all of them are busy, check which one is less busy
                //    List<EdgeModel> lessBusyEdges = (await redisApiClient.GetLessBusyEdgesAsync(connectedEdges)).ToList();
                //    tempDic.Add(policy.Id, lessBusyEdges);
                //    actionParam.Placement = lessBusyEdges.First();
                //}
                //else
                //{
                //    //tempDic.Add(policy.Id, freeEdges);
                //    //actionParam.Placement = freeEdges.First();
                // }
            }
            if (policy.PolicyName == "StoreAllInRobot")
            {
                // Put all neccesary containers inside robot
                actionParam.Placement = robot.Name;//guid
            }
            //All stored in the robot.

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

        // Get resource stadistic of failed action locations.




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